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
user@ubuntu:~$ mkdir -p catkin_ws/src
user@ubuntu:~$ cd catkin_ws/src
user@ubuntu:~/catkin_ws/src$ git clone --recurse-submodules https://github.com/rokokoo/elfin_manipulator.git
```

### ROS packages

```bash
user@ubuntu:~/catkin_ws$ rosdep install --from-paths src --ignore-src --rosdistro=melodic -y

user@ubuntu:~/catkin_ws$ sudo apt install ros-melodic-moveit-simple-controller-manager
```

The submodules that are used:

- [isura / elfin_robot](https://github.com/isura/elfin_robot/tree/melodic-devel), melodic-devel branch
- [roboticsgroup / roboticsgroup_gazebo_plugins](https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git)
- [rokokoo / robotiq](https://github.com/rokokoo/robotiq), this is a fork from ros-industrial, because we had to change some parameters of the gripper geometry.

Build the package using `catkin_make`

## Testing

Run start_gazebo.launch first

`user@ubuntu:~/catkin_ws/$ roslaunch samk_2f_140_gripper start_gazebo.launch`

After everything loads, start MoveIt with Rviz on a new terminal.

`user@ubuntu:~/catkin_ws/$ roslaunch samk_2f_140_gripper start_moveit.launch`

When the launch file finishes loading, you can start planning and executing movements.

## How to create a arm + manipulator combo

You can find a simple MoveIt tutorial [here](doc/create_moveit_config.md), that explains how to create a simple arm + manipulator pair, how to setup it to work with MoveIt trajectory and getting it to work in Gazebo.

## How to pull and run the Docker image

To download the image, run this command:
`docker pull ghcr.io/rokokoo/elfin_manipulator:1.0`

To use the image, run 
`docker run -it ghcr.io/rokokoo/elfin_manipulator:1.0`, but to get a graphical user-interface, you need to use [Rocker](https://github.com/osrf/rocker)
