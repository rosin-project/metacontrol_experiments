 #!/bin/bash

## Define path for workspaces (needed to run reasoner and metacontrol_sim in different ws)
## You need to create a "config.sh file in the same folder defining your values for these variables"

# Declare an array of string with the navigation profile
#declare -a NavProfile=("fast" "standard" "safe")

# count n simulations

declare count_simulations=1

# Declare reconfiguration (true or false)
# declare Reconfigurations=("false" "true")

# Declare reconfiguration (true or false)
component=$3

#for time_step in $(seq 1 50)
# for reconfiguration in ${Reconfigurations[@]};
# do
# for component in ${ComponentErrors[@]};
# do
if [[ $2 == "false" ]]
then
	if [[ $component == "true" ]]
	then
		continue 	# Continue at the "outer loop".
	fi
fi
obstacles=$4
increase_power=$5
		# for goal_position in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26
		#for goal_position in 14 
		for goal_position in 2
		do
		#	for init_position in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26
			#for init_position in 2 
			for init_position in 16 14 18 20
			do
				if [[ $goal_position -eq $init_position ]]; then
					continue # Continue at loop on 2nd level, that is "outer loop".
				fi
				echo "######################################################"
				echo "#                                                    #"
				echo "#  Running simulation NR: $count_simulations OF 27   #"
				echo "#                                                    #"
				echo "######################################################"

				bash -ic "./run_single_sim.sh -i $init_position -g $goal_position -n $1 -o $obstacles -p $increase_power -r $2 -k $component -l 1.0;
				exit;"
				count_simulations=$((count_simulations+1))
			done
		done
		echo "Moving logs to folder $(rospack find metacontrol_experiments)/ros/metacontrol_ws/src/metacontrol_experiments/data/R$2/C$component\_O$obstacles\_P$increase_power\_N$1"
		mkdir -p $(rospack find metacontrol_experiments)/data/R$2/C$component\_O$obstacles\_P$increase_power\_N$1
		tail -n 1 -q $(rospack find metacontrol_experiments)/data/*.csv >> $(rospack find metacontrol_experiments)/data/R$2\_C$component\_O$obstacles\_P$increase_power\_N$1.csv
		mv $(rospack find metacontrol_experiments)/data/*.csv $(rospack find metacontrol_experiments)/data/R$2/C$component\_O$obstacles\_P$increase_power\_N$1/
# done
# done
# done
# done
	# echo "Moving logs to folder $HOME/ros/metacontrol_ws/src/metacontrol_experiments/data/$nav_profile"
	# mkdir -p $HOME/ros/metacontrol_ws/src/metacontrol_experiments/data/$nav_profile
	# mv $HOME/ros/metacontrol_ws/src/metacontrol_experiments/data/*.csv $HOME/ros/metacontrol_ws/src/metacontrol_experiments/data/$nav_profile
# done
