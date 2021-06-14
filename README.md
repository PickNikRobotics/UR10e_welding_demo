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
vcs import < UR10e_welding_demo/UR10e_welding_demo.repos
rosdep install -r --from-paths . --ignore-src --rosdistro foxy -y
```

Build
```sh
cd $COLCON_WS
colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
