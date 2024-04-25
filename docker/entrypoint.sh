#!/bin/bash
set -e

if [[ $1 == "config" ]]
then
	cd "/root/exomy_ws/src/exomy/scripts"
	bash
elif [[ $1 == "autostart" ]]
then
	cd "/root/exomy_ws"

	source "/opt/ros/foxy/setup.bash"
	colcon build
	source "/root/exomy_ws/install/setup.bash"
	
	# start a rosbridge server in the background
	ros2 run rosbridge_server rosbridge_websocket_launch.xml &
	ros2 launch exomy exomy.launch.py
	# Sleep is needed to first print output and then start bash
	sleep 1
	
	cd "/root/exomy_ws"
	bash
elif [[ $1 == "devel" ]]
then
	cd "/root/exomy_ws"
	source "/opt/ros/foxy/setup.bash"
	
	bash
else
	bash
fi
