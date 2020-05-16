 #!/bin/bash

## Define path for workspaces (needed to run reasoner and metacontrol_sim in different ws)

export METACONTROL_WS_PATH="$HOME/ros/metacontrol_ws"
export REASONER_WS_PATH="$HOME/ros/reasoner_metacontrol_ws"
export PYTHON3_VENV_PATH="$HOME/ros/rospy3_melodic/"

## Define to store log files
declare LOG_DATA_PATH="$HOME/models_paper/log_data"

# Declare an array of string with the navigation profile 
declare -a NavProfile=("fast" "standard" "safe")

wait_for_gzserver_to_end () {

	for t in $(seq 1 100)
	do
		if test -z "$(ps aux | grep gzserver | grep -v grep )" 
		then
			echo "gzserver not running"
			break
		else
			echo "gzserver still running"
		fi
		sleep 1
	done
}

kill_running_ros_nodes () {
	# Kill all ros nodes that may be running 
	for i in $(ps aux | grep ros | grep -v grep | awk '{print $2}')
	do
		echo "kill -2 $i"
		kill -2 $i;
	done
	sleep 1
}


# Check that there are not running ros nodes
kill_running_ros_nodes
# If gazebo is running, it may take a while to end
wait_for_gzserver_to_end

#for time_step in $(seq 1 50)
for init_position in 1
do
	# Get x and y initial position from yaml file - takes some creativity :)
	declare init_pos_x=$(cat $METACONTROL_WS_PATH/src/metacontrol_experiments/yaml/initial_positions.yaml | grep S$init_position -A 5 | tail -n 1 | cut -c 10-)
	declare init_pos_y=$(cat $METACONTROL_WS_PATH/src/metacontrol_experiments/yaml/initial_positions.yaml | grep S$init_position -A 6 | tail -n 1 | cut -c 10-)
	# for goal_position in $(seq 1 50)
	for goal_position in 1
	do

		cat $METACONTROL_WS_PATH/src/metacontrol_experiments/yaml/goal_positions.yaml | grep G$goal_position -A 12 | tail -n 12 > $METACONTROL_WS_PATH/src/metacontrol_sim/yaml/goal.yaml
		#for nav_profile in ${NavProfile[@]};
		#do
	
			echo "Goal position: $goal_position - Initial position  $init_position - Navigation profile: $nav_profile"
					
			echo "Launching: MVP metacontrol world.launch"
			gnome-terminal --window --maximize -- bash -c "source $METACONTROL_WS_PATH/devel/setup.bash; roscore; exit"
			#Needed to allow other launchers to recognize the roscore
			sleep 3
			gnome-terminal --window --maximize -- bash -c "source $METACONTROL_WS_PATH/devel/setup.bash;
			roslaunch metacontrol_sim MVP_metacontrol_world.launch nav_profile:=$nav_profile initial_pose_x:=$init_pos_x initial_pose_y:=$init_pos_y;
			exit"
			echo "Launching: mros reasoner"
			gnome-terminal --window --maximize -- bash -c "source $PYTHON3_VENV_PATH/venv3.6_ros/bin/activate;
			source $PYTHON3_VENV_PATH/devel/setup.bash;
			source $REASONER_WS_PATH/devel/setup.bash;
			roslaunch mros1_reasoner run.launch onto:=mvp.owl;
			exit"
			echo "Running log and stop simulation node"
			bash -ic "source $METACONTROL_WS_PATH/devel/setup.bash; rosrun metacontrol_experiments stop_simulation_node;"
			echo "Simulation Finished!!"
			# Check that there are not running ros nodes
			kill_running_ros_nodes
			# Wait for gazebo to end
			wait_for_gzserver_to_end
		#done
	done
done