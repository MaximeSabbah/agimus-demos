AGIMUS demo 08 collision avoidance
--------------------------------

This demo demonstrates hard collision constraints on a compliant torque controller robot. It shows online collision avoidance as robot only has a frame placement cost, state regularization and torque regularization. Collision avoidance happens at the level of MPC without any planner.

Expected behaviors:
- In the RViz there is a visible sphere in front of the robot, that marks a collision object.
- Robot moves from left to right with very low gains making is very compliant and susceptible to being pushed by a human.
- Collision pair is defined between the end effector and the sphere in front of the robot.
- Robot can me easily moved around, while it executes it's task of moving between points, while being impossible to be pished inside of the sphere showing the hard collision constraint.

### Dependencies

This demo requires source built of dependencies found in:
- [franka.repos](../franka.repos)
- [control.repos](../control.repos)
- [agimus_dev.repos](../agimus_dev.repos)

### Simulation

> [!NOTE]
> Gazebo simulation of Franka robots require very high frequency of the simulated environment, hence users might experience high CPU utilization or even errors in cases where older and less powerful computers are used.

To launch the demo run:

```bash
ros2 launch agimus_demo_08_collision_avoidance bringup.launch.py use_gazebo:=true use_rviz:=true
```
Expected result: Robot moves from left to right avoiding a virtual sphere.

### Real robot

First turn on the robot and unlock joint in the web-ui. Move the robot a safe position allowing for a full range of joint motion while avoiding collisions with environment and not posing any threat to safety of people around.

> [!CAUTION]
> Before starting the launch file make sure robot is in a safe position and has sufficient movement space for it's joints and is not likely to collide with anything. Ensure all spectators are in a safe distance from the machine, and **the operator can quickly reach the Emergency Button in case error occurs**!

> [!CAUTION]
> Position the robot facing forward, to avoid an abrupt motion during the initialization. In case the robot start in collision it might perform a very abrupt motion causing it to lock the joints. To avoid this happening move the robot to a collision free configuration that is not too far from the workspace.

Launch the demo

```bash
ros2 launch agimus_demo_08_collision_avoidance bringup.launch.py robot_ip:=<robot-ip> use_rviz:=true disable_collision_safety:=true
```
Expected result: Robot moving left and right with a virtual sphere in the center of the workspace.

### Position of the sphere

Position of the sphere is published by a [obstacle_pose_publisher](./agimus_demo_08_collision_avoidance/obstacle_pose_publisher.py) node. Currently poses of the obstacles are fixed in place by this node. In case dynamic obstacles are expected this node has to be reimplemented to support motion capture or a fixed schedule movement of the obstacle.

### RT-COSMIK moving obstacles

This demo can also consume RT-COSMIK collision capsules (`/rtcosmik/collision_markers`) and map them to moving geometries used by the controller (`obstacle_0_0`, `obstacle_1_0`, `obstacle_2_0`).

Launch with:

```bash
ros2 launch agimus_demo_08_collision_avoidance bringup.launch.py use_rtcosmik_obstacles:=true
```

In this mode:
- `agimus_controller_params_rtcosmik.yaml` is used.
- The node `rtcosmik_obstacle_pose_adapter` replaces `obstacle_pose_publisher`.
- RViz environment model contains links `obstacle_0`, `obstacle_1`, `obstacle_2` and the controller consumes moving geometry topics `obstacle_0_0`, `obstacle_1_0`, `obstacle_2_0`.


export ROS_DOMAIN_ID=29
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

ros2 launch agimus_demo_08_collision_avoidance bringup.launch.py   use_gazebo:=true use_rviz:=true use_rtcosmik_obstacles:=true publish_obstacle_root_bridge:=true

ros2 launch rtcosmik_ros start.launch.py cam_calib_path:="/root/workspace/RT-COSMIK/config/cam_params"