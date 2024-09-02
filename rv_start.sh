#!/bin/bash

launch_path=/home/imca/imca_vision
exec_path="rm_bringup"
launch_file="bringup_merge.launch.py"
cd $launch_path 
FOX_DEBUG=0

if [ $FOX_DEBUG = 1 ]; then
	gnome-terminal -- bash fox.sh
	fi

launchCommand(){
	. /opt/ros/humble/setup.sh 
   	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 
	source install/setup.bash 
}

bashCommand(){
	ros2 launch $exec_path $launch_file
}

while true; do
	launchCommand
	bashCommand
	sleep 2
done


