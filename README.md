# Using Elfin 5 manipulator with MoveIt! package

## Installation process

<!-- ```bash
sudo apt install ros-melodic-soem \
ros-melodic-moveit \
ros-melodic-joint-trajectory-controller \
ros-melodic-socketcan-interface
``` -->

### Github

```bash
git clone --recurse-submodules https://github.com/rokokoo/elfin_manipulator.git
```

### ROS packages

```bash
cd elfin_manipulator/src

rosdep install --from-paths src --ignore-src --rosdistro=melodic -y

sudo apt install ros-melodic-moveit-simple-controller-manager
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

## How to create a arm + manipulator combo

You can find a simple MoveIt tutorial [here](doc/create_moveit_config.md), that explains how to create a simple arm + manipulator pair, how to setup it to work with MoveIt trajectory and getting it to work in Gazebo.
