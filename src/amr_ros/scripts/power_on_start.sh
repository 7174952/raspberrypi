#!/bin/bash

sudo -S su <<EOF
mikuni
EOF

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

export ROS_IP=192.168.2.103
export ROS_MASTER_URI=http://192.168.2.101:11311

gnome-terminal -x rosrun amr_ros sonar
