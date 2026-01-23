from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path

from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    generate_include_launch,
    get_use_sim_time,
)

from agimus_demos_common.static_transform_publisher_node import (
    static_transform_publisher_node,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    
    franka_robot_launch = generate_include_launch(
        "franka_common_lfc.launch.py",
        extra_launch_arguments={
            "ee_id": "franka_hand_with_camera",
            "use_camera": "true",
        },
    )
    
    vision_type_arg = LaunchConfiguration("vision_type")
    vision_type = context.perform_substitution(vision_type_arg).lower()

    agimus_controller_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_06_regrasp"),
            "config",
            "agimus_controller_params.yaml",
        ]
    )
    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )
    agimus_controller_node = Node(
        package="agimus_controller_ros",
        executable="agimus_controller_node",
        parameters=[get_use_sim_time(), agimus_controller_yaml],
        output="screen",
        remappings=[("robot_description", "robot_description_with_collision")],
    )

    environment_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demo_06_regrasp"),
                        "urdf",
                        "environment.urdf.xacro",
                    ]
                ),
            ]
        ),
        value_type=str,
    )
    environment_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="environment_publisher",
        output="screen",
        remappings=[("robot_description", "environment_description")],
        parameters=[{"robot_description": environment_description}],
    )
    tf_node = static_transform_publisher_node(
        frame_id="robot_attachment_link",
        child_frame_id="world",
    )

    tf_node_2 = static_transform_publisher_node(
        frame_id="tless-obj_000023",
        child_frame_id="current_object",
    )
    tf_node_support_link = static_transform_publisher_node(
        frame_id="support_link",
        child_frame_id="base",
        xyz=["0.563", "-0.166", "0.780"],
        rot_xyzw=["0.000", "0.000", "1.000", "0.000"],
    )
    env_nodes = [tf_node_2, tf_node_support_link]
    if vision_type == "simulate_happypose":
        simulated_object_pose = [0.04, -0.14, 0.54, 0.078, 0.031, -0.71, 0.7]
        happypose_simulation_params = {
            "object_id": "tless-obj_000023",
            "base_name": "support_link",
            "camera_name": "camera_color_optical_frame",
            "object_pose_in_base_txyz": simulated_object_pose[:3],
            "object_pose_in_base_qxyzw": simulated_object_pose[3:],
        }
        happypose_simulation_node = Node(
            package="agimus_demos_common",
            executable="happypose_simulation",
            parameters=[get_use_sim_time(), happypose_simulation_params],
            output="screen",
        )
        env_nodes.append(happypose_simulation_node)
    # # add simulation of vision detection
    # if vision_type in ["simulate_happypose", "simulate_apriltag_det"]:
    #     simulated_object_pose = [0.15, -0.2, 1.05, 0.0, 0.0, 0.707, 0.707]
    #     if vision_type == "simulate_apriltag_det":
    #         simulated_object_pose_as_str = [str(val) for val in simulated_object_pose]
    #         tf_node_object_detection = static_transform_publisher_node(
    #             frame_id="support_link",
    #             child_frame_id="tless-obj_000031",
    #             xyz=simulated_object_pose_as_str[:3],
    #             rot_xyzw=simulated_object_pose_as_str[3:],
    #         )
    #         env_nodes.append(tf_node_object_detection)
    #     elif vision_type == "simulate_happypose":
    #         happypose_simulation_params = {
    #             "object_id": "tless-obj_000031",
    #             "base_name": "support_link",
    #             "camera_name": "camera_color_optical_frame",
    #             "object_pose_in_base_txyz": simulated_object_pose[:3],
    #             "object_pose_in_base_qxyzw": simulated_object_pose[3:],
    #         }
    #         happypose_simulation_node = Node(
    #             package="agimus_demos_common",
    #             executable="happypose_simulation",
    #             parameters=[get_use_sim_time(), happypose_simulation_params],
    #             output="screen",
    #         )

    trajectory_weights_yaml = (
        Path(get_package_share_directory("agimus_demo_06_regrasp"))
        / "config"
        / "trajectory_weights_params.yaml"
    )
    use_gazebo = LaunchConfiguration("use_gazebo")
    use_gazebo_bool = context.perform_substitution(use_gazebo).lower() == "true"
    regrasp_node = ExecuteProcess(
        cmd=[
            "xterm",
            "-hold",
            "-e",
            'bash -c "source /opt/ros/humble/setup.bash && '
            f'ros2 run agimus_demo_06_regrasp regrasp_node --ros-args -p use_sim_time:={use_gazebo_bool} --params-file {trajectory_weights_yaml}"',
        ],
        output="screen",
    )

    return [
        franka_robot_launch,
        wait_for_non_zero_joints_node,
        environment_publisher_node,
        tf_node,
        *env_nodes,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[
                    agimus_controller_node,
                    regrasp_node,
                ],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_franka_args() + [OpaqueFunction(function=launch_setup)]
    )