#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/robot_arm_ws/install/setup.bash
S=~/robot_arm_ws/src/robot_arm_description/scripts
if [[ " $* " == *" --dry "* ]]; then
  python3 $S/motion_planning/square_demo.py "$@"; exit
fi
python3 $S/common/clear_marks.py
python3 $S/square/square_guide.py
python3 $S/motion_planning/square_demo.py "$@"
