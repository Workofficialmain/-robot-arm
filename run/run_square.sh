#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/robot_arm_ws/install/setup.bash

S=~/robot_arm_ws/src/robot_arm_description/scripts
MP=$S/motion_planning

# Dry-run: validate trajectory without moving the robot
if [[ " $* " == *" --dry "* ]]; then
    cd "$MP" && python3 trajectory_publisher.py \
        --shape square \
        --size 120.0 \
        --blend-radius 4.0 \
        --duration 20 \
        --dry
    exit $?
fi

# Remove previous reference shapes and trails
python3 "$S/common/clear_marks.py"

# Spawn black rounded-square guide at the actual tracing height
python3 "$S/square/square_guide.py" \
    --cx 244.0 \
    --cy -60.0 \
    --cz 15.0 \
    --size 120.0 \
    --blend-radius 4.0 \
    --name ref_square

# Execute the five-stage square trajectory and collect actual TCP trail
cd "$MP" && python3 trajectory_publisher.py \
    --shape square \
    --size 120.0 \
    --blend-radius 4.0 \
    --duration 20 \
    --execute \
    --trail
