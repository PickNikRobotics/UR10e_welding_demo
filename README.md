# UR10e_welding_demo

Description: A UR10e welding application demo using online motion planning based on MoveIt's Hybrid Planning pipeline.

<img src="https://avatars.githubusercontent.com/u/155854?s=200&v=4" width="60"> &nbsp;&nbsp;&nbsp;&nbsp;<img src="https://picknik.ai/assets/images/logo.jpg" width="150">

Developed by [Fraunhofer IPA](https://www.ipa.fraunhofer.de/) and [PickNik Robotics](http://picknik.ai/)

# Install

Install in a new workspace:
```sh
export COLCON_WS=~/ws_ur10_welding/ # OR other name
mkdir -p $COLCON_WS/src
```

Get the code
```sh
cd $COLCON_WS/src
git clone https://github.com/PickNikRobotics/UR10e_welding_demo.git # Can get specific branch
vcs import < UR10e_welding_demo/upstream.repos
rosdep install -r --from-paths . --ignore-src --rosdistro foxy -y
```

Build
```sh
cd $COLCON_WS
colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

# Launch
### MoveIt Bringup
This will launch a simulated UR10e (in the right scene, with welding gun) with MoveIt2 and let you plan and execute around

```sh
ros2 launch ipa_bringup ur_control.launch.py ur_type:=ur10e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=true
```

### Hybrid Planning Demonstration
This will launch a simulated UR5e, plan a Cartesian "welding" path, and allow user input to twist the last joint during the path execution.

In a terminal:
```sh
ros2 launch ipa_bringup hybrid_planning_demo.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=true
```

Use the RViz Motion Planning widget to plan and execute to the state: `pre_welding_configuration`. In 2 more terminals, run:

```sh
ros2 launch ipa_bringup send_hybrid_goal.launch.py
```

```sh
ros2 run ipa_bringup servo_keyboard_input
```

As the robot starts the Cartesian path, you should be able to use the `6` key to rotate the last joint. `R` toggles the directon of rotation.
> Note: you must have the `servo_keyboard_input` terminal selected for it to send commands

### Option B: Launch demo with catmux
If you have not done it yet, install [catmux](https://github.com/fmauch/catmux)
```sh
pip3 install --user catmux
```
Run the demo with
```sh
cd YOUR_WORKSPACE_ROOT
source install/setup.bash
catmux_create_session $(ros2 pkg prefix ipa_bringup)/share/ipa_bringup/config/catmux_session_config.yaml>
```
