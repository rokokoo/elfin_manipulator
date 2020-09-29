# Using Elfin 5 manipulator with MoveIt! package

## Installation process

### ROS packages

```bash
sudo apt install ros-melodic-soem \
ros-melodic-moveit \
ros-melodic-joint-trajectory-controller \
ros-melodic-socketcan-interface
```

### Github
```bash
git clone --recurse-submodules https://github.com/rokokoo/elfin_manipulator.git
```

The submodules that are used:
- [isura / elfin_robot](https://github.com/isura/elfin_robot/tree/melodic-devel), melodic-devel branch
- [roboticsgroup / roboticsgroup_gazebo_plugins](https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git)
- [ros-industrial / robotiq](https://github.com/ros-industrial/robotiq.git)

Build the package using `catkin_make`

## Testing

Run start_gazebo.launch first

`roslaunch samk_2f_gripper start_gazebo.launch`

After everything loads, start MoveIt with Rviz on a new terminal.

`roslaunch samk_2f_gripper start_moveit.launch`

When the launch file finishes loading, you can start planning and executing movements.

# TODO

- Create a page, that contains information on how to make a new moveit config with a manipulator attached.