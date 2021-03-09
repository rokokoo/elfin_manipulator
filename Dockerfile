FROM osrf/ros:melodic-desktop

SHELL ["/bin/bash", "-c"]

RUN mkdir /workspace/elfin_ws/src -p

COPY sources.sh /workspace/elfin_ws/sources.sh

WORKDIR /workspace/elfin_ws/src

RUN git config --global user.name "someone" && git config --global user.email "someone@someplace.com"
#RUN git clone https://github.com/hans-robot/elfin_robot.git
RUN git clone --recurse-submodules https://github.com/rokokoo/elfin_manipulator.git

RUN apt-get -qq update && \
	apt-get -qq install -y \
	ros-melodic-moveit-simple-controller-manager \
	python-catkin-tools	\ 
	&& \
	rosdep update && \
	rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
	rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/elfin_ws/

#RUN /bin/bash -c 'source /opt/ros/melodic/setup.sh; catkin config --init --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON; catkin build elfin_ethercat_driver; catkin build --limit-status-rate 0.001 --no-notify'
RUN source /opt/ros/melodic/setup.sh && \
	catkin config --init --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && \
	catkin build elfin_ethercat_driver && \
	catkin build --limit-status-rate 0.001 --no-notify
