 #!/bin/bash

## Define path for workspaces (needed to run reasoner and metacontrol_sim in different ws)
## You need to create a "config.sh file in the same folder defining your values for these variables"

# Declare an array of string with the navigation profile
declare -a NavProfile=("fast" "standard" "safe")

# count n simulations

declare count_simulations=0

# Declare reconfiguration (true or false)
declare reconfiguration="false"

#for time_step in $(seq 1 50)
for init_position in 1 2 3 
do
	for goal_position in 1 2 3 
	do
		for nav_profile in ${NavProfile[@]};
		do
			for obstacles in 0 1 2 3
			do
				for increase_power in 0 1.1 1.2 1.3
				do
					bash -ic "./run_single_sim.sh -i $init_position -g $goal_position -n $nav_profile -o $obstacles -p $increase_power -r $reconfiguration;
					exit;"
					count_simulations=$((count_simulations+1))
					echo $count_simulations
				done
			done
		done
	done
done
