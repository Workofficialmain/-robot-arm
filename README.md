# robot_arm_description

## Folders
- urdf/     robot_arm.urdf (RViz), robot_arm_gz.urdf (Gazebo)
- launch/   gazebo.launch.py
- config/   controllers.yaml
- meshes/   STL parts
- scripts/circle/   circle_guide.py, circle_demo.py  (legacy/: first trajectory version)
- scripts/square/   square_guide.py, square_demo.py
- scripts/common/   clear_marks.py (removes all dots from Gazebo)
- scripts/setup/    make_urdf.py, make_gz_urdf.py, move_joints.py, freecad/ pivot scripts
- run/      start_gazebo.sh, run_circle.sh, run_square.sh

## Run
1. run/start_gazebo.sh              (leave open, wait for the arm)
2. ros2 control list_controllers    (both must be active)
3. run/run_circle.sh  or  run/run_square.sh
   add --dry to check reach and joint limits without moving

## Requirements
- Ubuntu Linux
- ROS 2 Humble
- Gazebo
- ROS 2 ros2_control and joint trajectory controller packages

## ROS Distribution
This project is developed for ROS 2 Humble.
