AGIMUS demo 05 pick and place
-----------------------------

The purpose of this demo is to use the AGIMUS architecture to unbox some objects from one box to another with visual servoing and collision avoidance.
Expected behaviors:
- The robot goes over a box of object.
- The robot start approaching one object.
- The robot goes to grasp an object.
- The robot move the object above a desired location.
- The robot drops the object.

### Dependencies

This demo requires source built of dependencies found in:
- [franka.repos](../franka.repos)
- [control.repos](../control.repos)
- [agimus_dev.repos](../agimus_dev.repos)


for the vision you'll also need to install the package [olt_ros2_pipeline](https://github.com/agimus-project/olt_ros2_pipeline?tab=readme-ov-file#launch-apriltag) from source.

### Visual Servoing
The visual servoing can be done either using apriltags or happypose, the parameter vision_type allows to choose between them and whether it's in simulation or not. In experiment, you'll also have to launch the vision by using olt_ros2_pipeline package.

### Apriltag setup
the apriltag used can be found  [here](https://triagechallenge.darpa.mil/docs/AprilTag_0-20_family36h11.pdf),
the apriltag side size used was 44 mm, but can be modified [here](https://github.com/agimus-project/olt_ros2_pipeline/blob/topic/tmartinez/apriltag/config/tags_36h11.yaml#L22)
We used a maizena box with dimensions 0.11 * 0.048 * 0.166 (m³) for this demo when using apriltag.
In the code the object is introduced as the tless_obj-31 to add it more easily in the architecture.

### Available tless objects for happypose
Five tless objects can be used with happypose in this demo their tless numbers are 21, 22, 23, 25, 26.

## Start the demo in simulation using the Panda robot.
For Apriltag detection:

```bash
ros2 launch agimus_demo_05_pick_and_place bringup.launch.py vision_type:=simulate_apriltag_det use_gazebo:=true gz_headless:=true use_rviz:=true
```
After the xterm terminal is opened, type there `o.pick_and_place('obj_<tless object number>')`. For Apriltag the tless object number is 31.

## Start the demo on hardware using the Panda robot.
To place the source and destination box as it is in hpp, you may have first to calibrate the position of the two boxes, for that you can use in the xterm terminal `o.calibrate`.

To start the demo (dual setup + apriltag) :
```bash
gepetto-gui
```

```bash
ros2 launch olt_ros2_pipeline realsense_apriltag.launch.py
```

```bash
ros2 launch agimus_demo_05_pick_and_place bringup.launch.py arm_id:=fer vision_type:=apriltag_det robot_ip:=172.17.1.3 aux_computer_ip:=panda2 aux_computer_user:=msabbah  use_ft_sensor:=false use_rviz:=true
```

To start the demo (dual setup + happypose for tless) :

In vision_cuda docker: 

Don't forget to add the preparation of the meshes: 

```bash
ros2 run m3t_tracker_ros prepare_sparse_views         --input-path $HAPPYPOSE_DATA_DIR/bop_datasets/tless/models_cad         --output-path $M3T_DATA_DIR         --use-depth
```

```bash
ros2 launch olt_ros2_pipeline happypose.launch.py dataset_name:=tless model_type:=pbr
```

```bash
ros2 launch olt_ros2_pipeline separate_nodes_pipeline.launch.py m3t_data_dir:=/tmp/m3t_data_dir
```

In control docker: 

```bash
gepetto-gui
```

```bash
ros2 launch agimus_demo_05_pick_and_place bringup.launch.py arm_id:=fer vision_type:=happypose robot_ip:=172.17.1.3 aux_computer_ip:=panda2 aux_computer_user:=msabbah  use_ft_sensor:=false use_rviz:=false
```

## tips
When hpp doesn't find a trajectory, looking at what the scene looks like for him in gepetto-gui is helpful, you can do that in the xterm terminal with
`v = o.hpp_client.vf.createViewer()`
`v = (o.hpp_client.q_init)`
you may find transformation or vision issues this way.

<!-- or 

in two terminals

```bash
ros2 launch realsense2_camera rs_launch.py
```

```bash
ros2 run apriltag_ros apriltag_node --ros-args   -r image_rect:=/camera/camera/color/image_raw   -r camera_info:=/camera/camera/color/camera_info   --params-file /home/gepetto/ros2_ws/src/olt_ros2_pipeline/config/tags_36h11.yaml
```

To check that the tf is well streamed
```bash
ros2 run tf2_ros tf2_echo camera_color_optical_frame tless-obj_000031
``` -->