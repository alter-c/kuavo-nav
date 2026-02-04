#!/bin/bash
source ~/kuavo-ros-opensource/devel/setup.bash
# roslaunch humanoid_plan_arm_trajectory humanoid_plan_arm_trajectory.launch
python3 ./bin/plan_arm_traj.py --action $1 --mode $2
