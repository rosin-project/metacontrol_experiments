
# metacontrol_experiments

This package runs and controls batch simulations of the [metacontrol_sim](https://github.com/rosin-project/metacontrol_sim) package, it can create contingencies that should result in reconfigurations of the navigation stack.

## Setup using wstool

**Note** This step will include all the necessary packages, including those listed in the [metacontrol_sim](https://github.com/rosin-project/metacontrol_sim) package. Please make sure you're not duplicating them.

In the following instructions, we assume that the ROS workspace is called `metacontrol_ws`, but you can give it any name you want.

```console
mkdir -p ~/metacontrol_ws/src
cd ~/metacontrol_ws
wstool init src https://raw.githubusercontent.com/rosin-project/metacontrol_experiments/master/metacontrol_experiments.rosinstall
rosdep install --from-paths ./src -y -i -r --skip-keys="abb_rws_interface"
```

Check that all dependencies are installed correctly, please make an issue if you find any errors.

## Build the code

The code can be built using the standard `catkin build` process.

```console
cd ~/metacontrol_ws
source /opt/ros/melodic/setup.bash
catkin build
source devel/setup.bash
```

## WS Path  variables

To run the scripts, you need to create a file named `config.sh`, in the same folder as the `run.sh` defining your values for the workspace path variable

An example, if you are using the same workspace names as above, would be:

```bash
#!/bin/bash

METACONTROL_WS_PATH=~/metacontrol_ws
```

## Run the scripts

There are two scripts, one to run a single simulation and another to run simulations on a batch.

### Run a single simulation

The ./run_single_sim.sh accepts the following parameters

```bash
-i <init_position: (1 / 2 / 3)>
-g <goal_position: (1 / 2 / 3)>
-n <nav_profile: ("fast" / "standard" / "safe" or fX_vX_rX)>
-r <run mros reconfiguration: ("true" / "false")>
-o <add obstacles: (0 / 1 / 2 / 3)>
-p <increase_power: ([0 -  2.0])>
-b <record rosbags: ("true" / "false")>
-e <nfr energy threshold : ([0 - 1])>
-s <nfr safety threshold : ([0 - 1])>
-c <close reasoner terminal after execution : ('true' / 'false')>"
```

If no parameters are given the first value is used byr default

#### Example of single run

This launch should result in one or two reconfigurations

```console
./run_single_sim.sh -i 1 -g 1 -n "f1_v2_r2" -o 2 -r "true" -p 1.7 -c "false"
```

#### Expected outcome

The script opens three additional terminals and one rviz window

  1. roscore
  1. Gazebo simulation with MVP metacontrol world.launch
  1. mros reasoner

The output of the main console (where the script is executed) should be like this

```console
$ ./run_single_sim.sh -i 1 -g 1 -n "f1_v2_r2" -o 2 -r "true" -p 1.7 -c "false"
Make sure there is no other gazebo instances or ROS nodes running:
kill -2 6644
kill -2 6884
kill -2 7017

Start a new simulation - Goal position: 1 - Initial position  1 - Navigation profile: f1_v2_r2

Launch roscore
Launching: MVP metacontrol world.launch
Launching: mros reasoner
Running log and stop simulation node
... logging to /home/mario/.ros/log/23173622-d894-11ea-a643-3448ed792014/roslaunch-mario-laptop-7964.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://mario-laptop:45471/

SUMMARY
========

PARAMETERS
 * /create_unknown_obstacles_node/goal_nr: 1
 * /create_unknown_obstacles_node/obstacles: 2
 * /rosdistro: melodic
 * /rosversion: 1.14.6
 * /stop_simulation_node/data_log_folder: /home/mario/ros/m...
 * /stop_simulation_node/increase_power_factor: 1.7
 * /stop_simulation_node/store_data_freq: 2.0

NODES
  /
    create_unknown_obstacles_node (metacontrol_experiments/create_unknown_obstacles.py)
    stop_simulation_node (metacontrol_experiments/stop_simulation_node)

ROS_MASTER_URI=http://localhost:11311

process[stop_simulation_node-1]: started with pid [8035]
process[create_unknown_obstacles_node-2]: started with pid [8038]
[ INFO] [1596794138.256581525]: [STOP SIM] - Start node
[ INFO] [1596794138.272768379]: [STOP SIM] Will increase power consumption by 1.70 after 0.6 of the path is completed
[ INFO] [1596794138.284240283]: [STOP SIM] Logging to /home/mario/ros/metacontrol_ws/src/metacontrol_experiments/data/log_Metacontrol_sim_07-11_55_38.csv
[ INFO] [1596794138.284347624]: [STOP SIM] Node Initialization Completed
[ INFO] [1596794138.284372739]: Initialized an async multi-thread node.
[ INFO] [1596794138.284544551]: [STOP SIM] - Wait for goal msg
[ INFO] [1596794143.261260103]: [LogData :: goal_callback] - Goal Pos x -8.500000 - y 7.000000
[ INFO] [1596794143.284665200]: [STOP SIM] - Initial goal distance: 19.10
[ INFO] [1596794143.284697029]: [STOP SIM] - Wait for robot to reach the goal
[INFO] [1596794144.564601]: Adding obstacles: 2
[INFO] [1596794144.598565]: Adding obstacles.
[INFO] [1596794144.647856]: SpawnModel: Successfully spawned entity
[INFO] [1596794144.870407]: SpawnModel: Successfully spawned entity
[create_unknown_obstacles_node-2] process has finished cleanly
log file: /home/mario/.ros/log/23173622-d894-11ea-a643-3448ed792014/create_unknown_obstacles_node-2*.log
[ INFO] [1596794178.584661119]: [STOP SIM] - 2 / 3 of the route completed
[ INFO] [1596794178.585825909]: Power consumption increased  
[ INFO] [1596794203.200138944]: [LogData :: reconfig_callback] - New configuration requested f1_v2_r1
[ INFO] [1596794207.149323292]: [LogData :: goal_callback] - Goal Pos x -8.500000 - y 7.000000
[ INFO] [1596794210.412765830]: [LogData :: reconfig_callback] - New configuration requested f1_v1_r1
[ INFO] [1596794214.475655073]: [LogData :: goal_callback] - Goal Pos x -8.500000 - y 7.000000
[ INFO] [1596794218.887366611]: [STOP SIM :: stop_simulation] Killing nodes
killing /fake_localization
killed
================================================================================REQUIRED process [stop_simulation_node-1] has died!
process has finished cleanly
log file: /home/mario/.ros/log/23173622-d894-11ea-a643-3448ed792014/stop_simulation_node-1*.log
Initiating shutdown!
================================================================================
[stop_simulation_node-1] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done
exit
Simulation Finished!!
kill -2 6644
kill -2 6884

```

##### Notes

- At the beggining of the script, all previous gazebo and ROS nodes are killed, if there is a gazebo process running or lagging, this may take some time and a message `-- gzserver still running` may appear.

- The `===REQUIRED process [stop_simulation_node-1] has died!` part, appears red, but it's not an error, is due to the process being terminated at the end of the simulation.

- RViz and the three terminals should close at the end of the simulation. However, if the `-c "false"` option is given, the `mros_reasoner` terminal will wait for an external input before closing.

### Run simulations in batch

Edit the for loops inside `./run_batch_sim.sh` to define set of parameters and whether or not to run reconfiguration

Run the script

```console
./run_batch_sim.sh
```

## Getting logs

Two type of logs are available

### csv files

The `stop_simulation_node` subscribes to the topics containing the relevant info for the logs, it extracts the corresponding info (i.e. corresponding `key` value from the `diagnostics` topic for the QA) and stores them in a `.csv` at a given frequency as

- The default storage folder is `"$(find metacontrol_experiments)/data/"`
- The storage folder can be modified in the `launch/stop_simulation.launch` file
- Store frequency can be modified in the `launch/stop_simulation.launch` file ( 0 means no recording)

### rosbags

- `.bag` files can be stored if the parameter -b true is given to the `run_single_sim.sh` scripts
- The default storage folder is `"$(find metacontrol_experiments)/bags/"`
- The storage folder can be modified in the `launch/stop_simulation.launch` file
