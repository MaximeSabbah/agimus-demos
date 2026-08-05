"""Compare Hydrax Feedback-MPPI gains with Crocoddyl Riccati gains.

The input NPZ is produced by
``validation/export_panda_feedback_lq.py`` in MaximeSabbah/hydrax. It contains
an exact local linear-quadratic approximation of the Hydrax MJX rollout, so
this script compares the gain algorithms without introducing a MuJoCo versus
Pinocchio model mismatch.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import crocoddyl
import matplotlib.pyplot as plt
import numpy as np


def _symmetrize(matrix: np.ndarray) -> np.ndarray:
    return 0.5 * (matrix + matrix.T)


def _regularize_hessian(
    matrix: np.ndarray,
    minimum_eigenvalue: float,
) -> tuple[np.ndarray, float]:
    matrix = _symmetrize(matrix)
    current_minimum = float(np.linalg.eigvalsh(matrix).min())
    shift = max(0.0, minimum_eigenvalue - current_minimum)
    if shift > 0.0:
        matrix = matrix + shift * np.eye(matrix.shape[0])
    return matrix, shift


def _numpy_riccati(
    A: np.ndarray,
    B: np.ndarray,
    Q: np.ndarray,
    R: np.ndarray,
    N: np.ndarray,
    Qf: np.ndarray,
) -> list[np.ndarray]:
    """Reference finite-horizon Riccati recursion using Crocoddyl's sign."""
    P = _symmetrize(Qf)
    gains: list[np.ndarray] = []
    for index in range(A.shape[0] - 1, -1, -1):
        Qxx = _symmetrize(Q[index] + A[index].T @ P @ A[index])
        Quu = _symmetrize(R[index] + B[index].T @ P @ B[index])
        Qux = N[index].T + B[index].T @ P @ A[index]
        gain = np.linalg.solve(Quu, Qux)
        P = _symmetrize(Qxx - Qux.T @ gain)
        gains.append(gain)
    gains.reverse()
    return gains


def _crocoddyl_gain(
    A: np.ndarray,
    B: np.ndarray,
    Q: np.ndarray,
    R: np.ndarray,
    N: np.ndarray,
    Qf: np.ndarray,
) -> tuple[np.ndarray, list[np.ndarray]]:
    nx = A.shape[1]
    nu = B.shape[2]
    zeros_x = np.zeros(nx)
    zeros_u = np.zeros(nu)

    running_models = [
        crocoddyl.ActionModelLQR(
            A[index],
            B[index],
            Q[index],
            R[index],
            N[index],
            zeros_x,
            zeros_x,
            zeros_u,
        )
        for index in range(A.shape[0])
    ]
    terminal_model = crocoddyl.ActionModelLQR(
        np.eye(nx),
        np.zeros((nx, nu)),
        Qf,
        np.eye(nu),
        np.zeros((nx, nu)),
        zeros_x,
        zeros_x,
        zeros_u,
    )
    problem = crocoddyl.ShootingProblem(
        np.zeros(nx),
        running_models,
        terminal_model,
    )
    solver = crocoddyl.SolverFDDP(problem)
    xs = [np.zeros(nx) for _ in range(A.shape[0] + 1)]
    us = [np.zeros(nu) for _ in range(A.shape[0])]
    solver.setCandidate(xs, us, True)
    solver.computeDirection()
    gains = [np.asarray(gain).copy() for gain in solver.K]
    return gains[0], gains


def _relative_error(actual: np.ndarray, reference: np.ndarray) -> float:
    denominator = max(float(np.linalg.norm(reference)), 1e-12)
    return float(np.linalg.norm(actual - reference) / denominator)


def _cosine_similarity(left: np.ndarray, right: np.ndarray) -> float:
    denominator = float(np.linalg.norm(left) * np.linalg.norm(right))
    if denominator < 1e-12:
        return float("nan")
    return float(np.vdot(left.ravel(), right.ravel()) / denominator)


def _snapshot_metrics(
    K_feedback: np.ndarray,
    K_crocoddyl: np.ndarray,
    A0: np.ndarray,
    B0: np.ndarray,
) -> dict[str, float]:
    # Crocoddyl stores K for delta_u = -K delta_x. Hydrax stores the signed
    # derivative directly for delta_u = K_feedback delta_x.
    K_crocoddyl_signed = -K_crocoddyl
    nq = K_feedback.shape[1] // 2
    return {
        "relative_frobenius_error": _relative_error(
            K_feedback, K_crocoddyl_signed
        ),
        "relative_position_block_error": _relative_error(
            K_feedback[:, :nq], K_crocoddyl_signed[:, :nq]
        ),
        "relative_velocity_block_error": _relative_error(
            K_feedback[:, nq:], K_crocoddyl_signed[:, nq:]
        ),
        "cosine_similarity": _cosine_similarity(
            K_feedback, K_crocoddyl_signed
        ),
        "feedback_mppi_norm": float(np.linalg.norm(K_feedback)),
        "crocoddyl_signed_norm": float(np.linalg.norm(K_crocoddyl_signed)),
        "feedback_mppi_closed_loop_radius": float(
            np.max(np.abs(np.linalg.eigvals(A0 + B0 @ K_feedback)))
        ),
        "crocoddyl_closed_loop_radius": float(
            np.max(np.abs(np.linalg.eigvals(A0 - B0 @ K_crocoddyl)))
        ),
    }


def _plot_summary(
    times: np.ndarray,
    metrics: list[dict[str, float]],
    output: Path,
) -> None:
    relative = np.asarray(
        [metric["relative_frobenius_error"] for metric in metrics]
    )
    position = np.asarray(
        [metric["relative_position_block_error"] for metric in metrics]
    )
    velocity = np.asarray(
        [metric["relative_velocity_block_error"] for metric in metrics]
    )

    figure, axis = plt.subplots(figsize=(8.0, 4.5))
    axis.plot(times, relative, marker="o", label="full K")
    axis.plot(times, position, marker="o", label="position block")
    axis.plot(times, velocity, marker="o", label="velocity block")
    axis.set_xlabel("reach time [s]")
    axis.set_ylabel("relative Frobenius error")
    axis.grid(True)
    axis.legend()
    figure.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=160)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Compare Feedback-MPPI gains against Crocoddyl's finite-horizon "
            "Riccati gains on the exact local Hydrax OCP approximation."
        )
    )
    parser.add_argument("input", type=Path)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("panda_feedback_crocoddyl_comparison.json"),
    )
    parser.add_argument(
        "--matrices-output",
        type=Path,
        default=None,
        help="Optional NPZ containing both gain sequences.",
    )
    parser.add_argument(
        "--plot",
        type=Path,
        default=None,
        help="Optional summary plot path.",
    )
    parser.add_argument(
        "--minimum-state-eigenvalue",
        type=float,
        default=0.0,
        help="Minimum eigenvalue enforced on each running Q Hessian.",
    )
    parser.add_argument(
        "--minimum-control-eigenvalue",
        type=float,
        default=1e-9,
        help="Minimum eigenvalue enforced on each running R Hessian.",
    )
    parser.add_argument(
        "--minimum-terminal-eigenvalue",
        type=float,
        default=0.0,
        help="Minimum eigenvalue enforced on Qf.",
    )
    args = parser.parse_args()

    archive = np.load(args.input, allow_pickle=False)
    metadata = json.loads(str(archive["metadata_json"]))
    times = np.asarray(archive["times"])
    K_feedback_all = np.asarray(archive["K_feedback_mppi"])

    K_crocoddyl_all = []
    K_numpy_all = []
    metrics = []
    diagnostics: list[dict[str, Any]] = []

    for snapshot_index, time_sec in enumerate(times):
        A = np.asarray(archive["A"][snapshot_index])
        B = np.asarray(archive["B"][snapshot_index])
        Q = np.asarray(archive["Q"][snapshot_index])
        R = np.asarray(archive["R"][snapshot_index])
        N = np.asarray(archive["N"][snapshot_index])
        Qf = np.asarray(archive["Qf"][snapshot_index])

        running_state_shifts = []
        running_control_shifts = []
        for node in range(R.shape[0]):
            Q[node], state_shift = _regularize_hessian(
                Q[node], args.minimum_state_eigenvalue
            )
            R[node], control_shift = _regularize_hessian(
                R[node], args.minimum_control_eigenvalue
            )
            running_state_shifts.append(state_shift)
            running_control_shifts.append(control_shift)
        Qf, terminal_shift = _regularize_hessian(
            Qf, args.minimum_terminal_eigenvalue
        )

        K_crocoddyl, K_crocoddyl_sequence = _crocoddyl_gain(
            A, B, Q, R, N, Qf
        )
        K_numpy_sequence = _numpy_riccati(A, B, Q, R, N, Qf)
        K_numpy = K_numpy_sequence[0]
        recursion_error = float(
            np.max(np.abs(K_crocoddyl - K_numpy))
        )
        if recursion_error > 1e-7:
            raise RuntimeError(
                "Crocoddyl and the independent Riccati recursion disagree: "
                f"max error={recursion_error:.3e} at t={time_sec:.3f} s"
            )

        K_feedback = K_feedback_all[snapshot_index]
        metric = _snapshot_metrics(K_feedback, K_crocoddyl, A[0], B[0])
        metric.update(
            {
                "time": float(time_sec),
                "gain_ess": float(archive["gain_ess"][snapshot_index]),
                "gain_nominal_weight": float(
                    archive["gain_nominal_weight"][snapshot_index]
                ),
                "crocoddyl_numpy_max_error": recursion_error,
            }
        )
        metrics.append(metric)
        diagnostics.append(
            {
                "time": float(time_sec),
                "running_Q_regularization": running_state_shifts,
                "running_R_regularization": running_control_shifts,
                "terminal_Q_regularization": terminal_shift,
                "minimum_Q_eigenvalue": float(
                    min(np.linalg.eigvalsh(matrix).min() for matrix in Q)
                ),
                "minimum_R_eigenvalue": float(
                    min(np.linalg.eigvalsh(matrix).min() for matrix in R)
                ),
                "minimum_Qf_eigenvalue": float(
                    np.linalg.eigvalsh(Qf).min()
                ),
            }
        )
        K_crocoddyl_all.append(np.stack(K_crocoddyl_sequence))
        K_numpy_all.append(np.stack(K_numpy_sequence))
        print(
            f"t={time_sec:.3f} s  rel_error="
            f"{metric['relative_frobenius_error']:.3f}  cosine="
            f"{metric['cosine_similarity']:.3f}  ESS="
            f"{metric['gain_ess']:.1f}"
        )

    report = {
        "metadata": metadata,
        "comparison_convention": (
            "K_feedback_mppi is compared with -K_crocoddyl because Hydrax "
            "applies delta_u = K_feedback_mppi delta_x while Crocoddyl "
            "applies delta_u = -K_crocoddyl delta_x."
        ),
        "regularization": {
            "minimum_state_eigenvalue": args.minimum_state_eigenvalue,
            "minimum_control_eigenvalue": args.minimum_control_eigenvalue,
            "minimum_terminal_eigenvalue": args.minimum_terminal_eigenvalue,
        },
        "snapshots": metrics,
        "diagnostics": diagnostics,
        "aggregate": {
            "mean_relative_frobenius_error": float(
                np.mean(
                    [metric["relative_frobenius_error"] for metric in metrics]
                )
            ),
            "max_relative_frobenius_error": float(
                np.max(
                    [metric["relative_frobenius_error"] for metric in metrics]
                )
            ),
            "mean_cosine_similarity": float(
                np.nanmean([metric["cosine_similarity"] for metric in metrics])
            ),
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(f"wrote {args.output}")

    if args.matrices_output is not None:
        args.matrices_output.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            args.matrices_output,
            times=times,
            K_feedback_mppi=K_feedback_all,
            K_crocoddyl=np.stack(K_crocoddyl_all),
            K_numpy=np.stack(K_numpy_all),
        )
        print(f"wrote {args.matrices_output}")

    if args.plot is not None:
        _plot_summary(times, metrics, args.plot)
        print(f"wrote {args.plot}")


if __name__ == "__main__":
    main()
