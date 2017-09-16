#!/bin/bash
# https://help.ubuntu.com/community/Beginners/BashScripting

clear
ID='HI'

cd ~/obj_grasp_ws/src/iai_trajectory_generation_boxy/src
i=0

for VAR in {0..7}
do
    echo "Setting initial robot config " $VAR
    python set_initial_pose.py $VAR
    sleep 1
    echo "\multicolumn{8}{|c|}{Initial robot configuration $VAR} \ \hline" >> "/home/mine/obj_grasp_ws/src/iai_trajectory_generation_boxy/test_scores/scores.tex"
    for OBJ in 'cup' 'mondamin' 'knorr_tomate'
    do
        ((++i))
        OBJID="$ID$i"
        echo "$i"
        echo $OBJ
        rostopic pub -1 /projected_grasping_server/goal iai_trajectory_generation_boxy/ProjectedGraspingActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: $OBJID
goal:
  object: "$OBJ
        sleep 1m
		  sleep 15
		  if [ "$OBJ" = "knorr_tomate" ]; then
				python set_initial_pose.py $(($VAR+1))
		  fi
		  sleep 2m
    done
	 
	 
    #pkill roslaunch
done



