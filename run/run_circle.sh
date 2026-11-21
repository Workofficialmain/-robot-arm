#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/robot_arm_ws/install/setup.bash

S=~/robot_arm_ws/src/robot_arm_description/scripts
MP=$S/motion_planning

# --dry: validate Cartesian trajectory and limits without moving the robot
if [[ " $* " == *" --dry "* ]]; then
  cd "$MP" && python3 trajectory_publisher.py --shape circle --cx 300.0 --cy 0.0 --radius 60.0 --pen 15.0 --dry
  exit
fi

# Remove previous reference shapes and trails from Gazebo
python3 "$S/common/clear_marks.py"

# Spawn continuous black reference ring at tracing height (~8 mm wide, 360 overlapping box segments)
# Center: (300, 0) mm, Radius: 60 mm, Height: 15.0 mm above ground (matching TCP tracing height)
python3 "$S/circle/circle_guide.py" \
    --cx 300.0 --cy 0.0 --cz 15.0 --radius 60.0 --name ref_circle

# Execute 5-stage circle trajectory: approach, descent, 360-deg floor circle tracing, retract, return home.
# Trail records actual /joint_states measurements strictly during Stage 3 and spawns continuous red trail.
cd "$MP" && python3 trajectory_publisher.py \
    --shape circle --cx 300.0 --cy 0.0 --radius 60.0 --pen 15.0 --execute --trail
