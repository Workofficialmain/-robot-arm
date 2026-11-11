#!/bin/bash
pkill -9 gzserver; pkill -9 gzclient
pkill -f robot_state_publisher; pkill -f spawn_entity; pkill -f move_joints
export GAZEBO_MODEL_DATABASE_URI=""
export QT_QPA_PLATFORM=xcb
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/robot_arm_ws/install/robot_arm_description/share
source /usr/share/gazebo/setup.sh
source /opt/ros/humble/setup.bash
source ~/robot_arm_ws/install/setup.bash
ros2 launch ~/robot_arm_ws/src/robot_arm_description/launch/gazebo.launch.py
