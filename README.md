# robotont\_gazebo

[![Build Status](https://travis-ci.com/robotont/robotont_gazebo.svg?branch=melodic-devel)](https://travis-ci.com/github/robotont/robotont_gazebo)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Dependencies

* [robotont_description](https://github.com/robotont/robotont_description)

* [robotont_nuc_description](https://github.com/robotont/robotont_nuc_description)

## Running the simulator

To run the simulator:

```bash
roslaunch robotont_gazebo gazebo.launch
```
The launch file has four arguments:

* model - chooses between a model with NUC and realsense and a model without them
    * default: robotont_gazebo_nuc 
    * options: robotont_gazebo_nuc, robotont_gazebo_basic, robotont_gazebo_lidar
* world - chooses which world to use
    * default: empty.world
    * options: empty.world, minimaze.world, bangbang.world, between.world, colors.world
* x_pos - chooses x coordinate of the world, controls where the robot will spawn, default: 0



For example, the following command will spawn the robot to a map called bangbang.world in position x=2 and the model that will be used is robotont_gazebo_nuc.
```bash
roslaunch robotont_gazebo gazebo.launch world:=$(rospack find robotont_gazebo)/worlds/bangbang.world model:=robotont_gazebo_nuc x_pos:=2 
```

To simply run the worlds without using the arguments:

*   ```bash
    roslaunch robotont_gazebo world_minimaze.launch
    ```
*   ```bash
    roslaunch robotont_gazebo world_bangbang.launch
    ```

*   ```bash
    roslaunch robotont_gazebo world_between.launch
    ```
*   ```bash
    roslaunch robotont_gazebo world_colors.launch
    ```






# ign gazebo simulation for Robotont

Simulation of robotont on ignition gazebo 
release: citadel verison 3.5

# Instalation
``` bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ignition-citadel
```

## Usefull commands:
``` bash
ign gazebo -h
ign gazebo --versions
ign topic -l 
ign topic -e -t /topic
```

## How to run some examples:
* ign gazebo camera_sensor.sdf
* ign gazebo shapes.sdf
* ign gazebo diff_driver.sdf

## Run robotont model
``` bash
cd ~/robotont_ws
ign gazebo ./src/robototnt_gazebo/ign_worlds/robotont.sdf
```

## Run robotont model with camera sensor
``` bash
cd ~/robotont_ws
ign gazebo ./src/robototnt_gazebo/ign_worlds/roboto_with_depth_camera.sdf
```

## Connect with ROS
In order to connect ignition gazebo with ros it is neccesary to have ros_ign_bridge package. 

* For some reason when I installed it with the binary installation procedure, the node runs but it does not publish any message on the topics. 
``` bash 
sudo apt install ros-noetic-ros-ign
```

* Using the source intallation procedure, it works fine.

``` bash 
sudo apt install ros-melodic-rqt-image-view libignition-common3-dev libignition-transport8-dev libignition-msgs5-dev
export IGNITION_VERSION=citadel
mkdir -p ~/robotont_ws/src
cd ~/robotont_ws/src
git clone https://github.com/osrf/ros_ign.git -b melodic
cd ~/robotont_ws
rosdep install --from-paths src -i -y --rosdistro melodic \
  --skip-keys=ignition-gazebo2 \
  --skip-keys=ignition-gazebo3 \
  --skip-keys=ignition-msgs4 \
  --skip-keys=ignition-msgs5 \
  --skip-keys=ignition-rendering2 \
  --skip-keys=ignition-rendering3 \
  --skip-keys=ignition-sensors2 \
  --skip-keys=ignition-sensors3 \
  --skip-keys=ignition-transport7 \
  --skip-keys=ignition-transport8

  # Source ROS distro's setup.bash
source /opt/ros/melodic/setup.bash

catkin build
cd ~/robotont_ws
source devel/setup/bash

```

export SDF_PATH="~/robotont_ws/src/robotont_gazebo/ign_worlds"
export IGN_FILE_PATH="~/robotont_ws/src/robotont_gazebo/ign_worlds"


### Move the robot:
To command the robot directly using the ignition topics:
ign topic -t "/model/robotont/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.0}"


To connect with ros:
1. roscore
2. rosrun ros_ign_bridge parameter_bridge /model/robotont/cmd_vel@geometry_msgs/Twist@ignition.msgs.Twist
3. rostopic pub /model/robotont/cmd_vel geometry_msgs/Twist ...

### Camera:
ignition gazebo side: 
cd catking_ws that contains the robotont packages
1. ign gazebo ./src/robototnt_gazebo/ign_worlds/robotont_with_depth_camera.sdf
2. ign topic -l 
/camera_info
/depth_camera 
Dont forget to run the simulation on ignition gazebo (press play button)

ros side: 
1. roscore
2. rosrun ros_ign_bridge parameter_bridge /depth_camera@sensor_msgs/Image@ignition.msgs.Image
3. rostopic list 
it should appear the /depth_camera topic
4. rqt_image_view      
Update the window with the topic /depth_camera


References:
* http://sdformat.org/spec
* http://sdformat.org/tutorials
* https://github.com/ignitionrobotics/ros_ign/tree/melodic/ros_ign_bridge#prerequisites
* https://ignitionrobotics.org/libs/gazebo
