# robotont_gazebo

## 2D mapping

* Run the simulator
```roslaunch robotont_gazebo gazebo_teleop_keyboard.launch world:=$(rospack find robotont_gazebo)/worlds/mapping_new.world```

* Launch 2d_nav_carto_gazebo.launch
```roslaunch robotont_gazebo 2d_nav_carto_gazebo.launch```

* Launch Rviz
```roslaunch robotont_gazebo display_2d_mapping.launch ```
