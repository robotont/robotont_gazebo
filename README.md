# robotont\_gazebo

[![Build Status](https://travis-ci.com/github/robotont/robotont_gazebo.svg?branch=melodic-devel)](https://travis-ci.com/github/robotont/robotont_gazebo)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Running the simulator

To run the simulator:

```bash
roslaunch robotont_gazebo gazebo.launch
```
The launch file has four arguments:

* model - chooses between a model with NUC and realsense and a model without them
    * default: robotont_gazebo_nuc 
    * options: robotont_gazebo_nuc, robotont_gazebo_basic
* world - chooses which world to use
    * default: empty.world
    * options: empty.world, minimaze.world, bangbang.world, between.world, colors.world
* x_pos - chooses x coordinate of the world, controls where the robot will spawn, default: 0
* driver_hack: chooses whether a driver_hack will be used or not, is necessary if the robot will turn very slowly while using the simulator
    * default: true
    * options: true, false


For example, the following command will spawn the robot to a map called bangbang.world in position x=2, the model that will be used is robotont_gazebo_nuc and driver_hack will be on.

```bash
roslaunch robotont_gazebo gazebo.launch world:=$(rospack find robotont_gazebo)/worlds/bangbang.world model:=robotont_gazebo_nuc x_pos:=2 driver_hack:=true
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
