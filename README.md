# robotont\_gazebo

## Running the simulator

To run the simulator:

```bash
roslaunch robotont_gazebo gazebo_teleop_keyboard.launch
```
There are two arguments on the launch file, that change the map and the position, where the robot will spawn: 
* world - the map where the robot will spawn
* x_pos - x coordinate of the map, controls where the robot will spawn


For example, the following command will spawn the robot to a map called minimaze.world in position x=2:

```bash
roslaunch robotont_gazebo gazebo_teleop_keyboard.launch world:=$(rospack find robotont_gazebo)/worlds/bangbang.world x_pos:=2
```

To simply run the maps without using the arguments:

*   ```bash
    roslaunch robotont_gazebo gazebo_world_minimaze.launch
    ```

*   ```bash
    roslaunch robotont_gazebo gazebo_world_between.launch
    ```

*   ```bash
    roslaunch robotont_gazebo gazebo_world_bangbang.launch
    ```
