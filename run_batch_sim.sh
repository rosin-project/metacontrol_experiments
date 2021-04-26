 #!/bin/bash

## Define path for workspaces (needed to run reasoner and metacontrol_sim in different ws)
## You need to create a "config.sh file in the same folder defining your values for these variables"

# Declare an array of string with the navigation profile
#declare -a NavProfile=("fast" "standard" "safe")

# count n simulations

declare count_simulations=1

# Declare reconfiguration (true or false)
declare reconfiguration="true"

#for time_step in $(seq 1 50)
# for nav_profile in ${NavProfile[@]};
# do
# for goal_position in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26
for goal_position in 1 2
do
#	for init_position in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26
	for init_position in 1 2
	do
		for obstacles in 0
		do
			for increase_power in 0
			do
				echo "######################################################"
				echo "#                                                    #"
				echo "#  Running simulation NR: $count_simulations OF 27   #"
				echo "#                                                    #"
				echo "######################################################"

				bash -ic "./run_single_sim.sh -i $init_position -g $goal_position -n $1 -o $obstacles -p $increase_power -r $reconfiguration -l 1.0;
				exit;"
				count_simulations=$((count_simulations+1))
			done
		done
	done
done
	# echo "Moving logs to folder $HOME/ros/metacontrol_ws/src/metacontrol_experiments/data/$nav_profile"
	# mkdir -p $HOME/ros/metacontrol_ws/src/metacontrol_experiments/data/$nav_profile
	# mv $HOME/ros/metacontrol_ws/src/metacontrol_experiments/data/*.csv $HOME/ros/metacontrol_ws/src/metacontrol_experiments/data/$nav_profile
# done
