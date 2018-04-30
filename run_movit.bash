source install/setup.bash
roslaunch iiwa_moveit moveit_planning_execution.launch sim:=true  model:=iiwa14 hardware_interface:=EffortJointInterface robot_name:=ariac

