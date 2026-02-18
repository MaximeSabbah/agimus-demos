from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch.substitutions import Command, FindExecutable

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
    use_rtcosmik_obstacles_cfg = LaunchConfiguration("use_rtcosmik_obstacles")
    use_rtcosmik_obstacles = (
        use_rtcosmik_obstacles_cfg.perform(context).lower() == "true"
    )
    publish_world_frame_bridge_cfg = LaunchConfiguration("publish_world_frame_bridge")
    publish_world_frame_bridge = (
        publish_world_frame_bridge_cfg.perform(context).lower() == "true"
    )
    publish_obstacle_root_bridge_cfg = LaunchConfiguration(
        "publish_obstacle_root_bridge"
    )
    publish_obstacle_root_bridge = (
        publish_obstacle_root_bridge_cfg.perform(context).lower() == "true"
    )
    controller_params_file = (
        "agimus_controller_params_rtcosmik.yaml"
        if use_rtcosmik_obstacles
        else "agimus_controller_params.yaml"
    )
    obstacle_pose_remappings = (
        [
            ("obstacle_0_0", "/rtcosmik/collision_pose/right_upperarm"),
            ("obstacle_1_0", "/rtcosmik/collision_pose/right_lowerarm"),
            ("obstacle_2_0", "/rtcosmik/collision_pose/right_hand"),
        ]
        if use_rtcosmik_obstacles
        else []
    )

    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_08_collision_avoidance"),
            "rviz",
            "config.rviz",
        ]
    )

    franka_robot_launch = generate_include_launch(
        "franka_common_lfc.launch.py",
        extra_launch_arguments={"rviz_config_path": rviz_config_path},
    )

    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )

    agimus_controller_params = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_08_collision_avoidance"),
            "config",
            controller_params_file,
        ]
    )

    agimus_controller_node = Node(
        package="agimus_controller_ros",
        executable="agimus_controller_node",
        parameters=[
            get_use_sim_time(),
            agimus_controller_params,
        ],
        output="screen",
        remappings=[("robot_description", "/robot_description_with_collision")]
        + obstacle_pose_remappings,
    )

    environment_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demo_08_collision_avoidance"),
                        "urdf",
                        "environment.urdf.xacro",
                    ]
                ),
                " ",
                "use_rtcosmik_obstacles:=",
                use_rtcosmik_obstacles_cfg,
            ]
        ),
        value_type=str,
    )
    environment_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="environment_publisher",
        output="screen",
        parameters=[get_use_sim_time(), {"robot_description": environment_description}],
        remappings=[("robot_description", "environment_description")],
    )

    goal_publisher_params = (
        PathJoinSubstitution(
            [
                FindPackageShare("agimus_demo_08_collision_avoidance"),
                "config",
                "goal_publisher_params.yaml",
            ]
        ),
    )

    goal_publisher_node = Node(
        package="agimus_demo_08_collision_avoidance",
        executable="goal_publisher",
        name="goal_publisher_node",
        output="both",
        parameters=[
            get_use_sim_time(),
            ParameterFile(param_file=goal_publisher_params, allow_substs=True),
        ],
    )

    obstacle_pose_publisher_node = Node(
        package="agimus_demo_08_collision_avoidance",
        executable="obstacle_pose_publisher",
        name="obstacle_pose_publisher_node",
        output="both",
        parameters=[get_use_sim_time()],
    )
    rtcosmik_pose_debug_markers_node = Node(
        package="agimus_demo_08_collision_avoidance",
        executable="rtcosmik_pose_debug_markers",
        name="rtcosmik_pose_debug_markers_node",
        output="screen",
        parameters=[get_use_sim_time()],
    )
    moving_obstacle_providers = [] if use_rtcosmik_obstacles else [obstacle_pose_publisher_node]
    world_frame_bridge_node = static_transform_publisher_node(
        frame_id="fer_link0",
        child_frame_id="world",
    )
    obstacle_root_bridge_node = static_transform_publisher_node(
        frame_id="fer_link0",
        child_frame_id="obstacle_root",
    )

    nodes = [
        franka_robot_launch,
        environment_publisher_node,
        wait_for_non_zero_joints_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[
                    agimus_controller_node,
                    *moving_obstacle_providers,
                ],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=agimus_controller_node,
                on_start=TimerAction(
                    period=5.0,
                    actions=[goal_publisher_node],
                ),
            )
        ),
    ]
    if publish_world_frame_bridge:
        nodes.append(world_frame_bridge_node)
    if use_rtcosmik_obstacles and publish_obstacle_root_bridge:
        nodes.append(obstacle_root_bridge_node)
    if use_rtcosmik_obstacles:
        nodes.append(rtcosmik_pose_debug_markers_node)
    return nodes


def generate_launch_description():
    return LaunchDescription(
        generate_default_franka_args()
        + [
            DeclareLaunchArgument(
                "use_rtcosmik_obstacles",
                default_value="false",
                description=(
                    "Use RT-COSMIK collision capsules as moving obstacles "
                    "instead of the static demo obstacle publisher."
                ),
            ),
            DeclareLaunchArgument(
                "publish_world_frame_bridge",
                default_value="true",
                description=(
                    "Publish a static identity transform from fer_link0 to world "
                    "to bridge RT-COSMIK world-framed data with demo frames."
                ),
            ),
            DeclareLaunchArgument(
                "publish_obstacle_root_bridge",
                default_value="true",
                description=(
                    "Publish a static identity transform from fer_link0 to obstacle_root "
                    "to connect the RT-COSMIK environment robot_state_publisher tree."
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
