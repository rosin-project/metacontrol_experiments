 #!/bin/bash

## Define path for workspaces (needed to run reasoner and metacontrol_sim in different ws)
## You need to create a "config.sh file in the same folder defining your values for these variables"


declare -a NavProfile=("f1_v1_r3" "f1_v3_r1" "f2_v1_r3")
echo $NavProfile

#Start Location: S1, S3Goal Location: G1, G2C: slow: f1_v1_r3, fast: f1_v3_r1, safe: f2_v1_r3T1: <no_continдency, low, medium, high>,T2:<no_continдency,10%,20%,30%>

# count n simulations
declare count_simulations=1

# Declare reconfiguration (true or false)
declare Reconfigurations=("false" "true")

#for time_step in $(seq 1 50)
for init_position in 1 3
do
	for goal_position in 1 2
	do
		for obstacles in 0 1 2 3
		do
			for increase_power in 0 1.3 1.5
			do
				for nav_profile in ${NavProfile[@]};
				do
					for reconfiguration in ${Reconfigurations[@]};
					do
						echo "######################################################"
						echo "#                                                    #"
						echo "#  Running simulation NR: $count_simulations OF 288  #"
						echo "#                                                    #"
						echo "######################################################"

						bash -ic "./run_single_sim.sh -i $init_position -g $goal_position -n $nav_profile -o $obstacles -p $increase_power -r $reconfiguration;
						exit;"
						count_simulations=$((count_simulations+1))
						###
						# Update these paths to the correct values for the data folder
						###

						echo "Moving logs to folder $HOME/ros/metacontrol_ws/src/metacontrol_experiments/data/R$reconfiguration/I$init_position\_G$goal_position\_O$obstacles\_P$increase_power\_N$nav_profile"
						mkdir -p $HOME/ros/metacontrol_ws/src/metacontrol_experiments/data/R$reconfiguration/I$init_position\_G$goal_position\_O$obstacles\_P$increase_power\_N$nav_profile
						mv $HOME/ros/metacontrol_ws/src/metacontrol_experiments/data/*.csv $HOME/ros/metacontrol_ws/src/metacontrol_experiments/data/R$reconfiguration/I$init_position\_G$goal_position\_O$obstacles\_P$increase_power\_N$nav_profile/
					done
				done
			done
		done
	done
done


