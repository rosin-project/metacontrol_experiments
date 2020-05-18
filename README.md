# metacontrol_experiments
Controls simulation for metacontrol_sim

## WS Path  variables
create a `config.sh` file in the same folder as the `run.sh` defining your values for these variables

An example would be:

```
#!/bin/bash

METACONTROL_WS_PATH=/path/to/your/metacontrol_ws
REASONER_WS_PATH=/path/to/your/reasoner_ws
PYTHON3_VENV_PATH=/path/to/your/rospython3_ws
```

## Run the scripts

There are two scripts, one to run a single simulation and another to run a batch.

### Run a single simulation

The ./run_single_sim.sh accepts the following parameters

```
-i <init_position: (1 / 2 / 3)>
-g <goal_position: (1 / 2 / 3)>
-n <nav_profile: ("fast" / "standard" / "safe")>
-r <Run mros reconfiguration: ("true" / "false")>
-o <add obstacles: (0 / 1 / 2 / 3)>
-p <increase_power: (0/1.1/1.2/1.3)>
```

If no parameters are given the default values are used

#### Example of single run

```
./run_single_sim.sh -p 1 -g 2 -n "safe" -o 2 -r "true" -p 1.2
```

### Run simulations in batch

Edit the for loops inside `./run_batch_sim.sh` to define set of parameters and whether or not to run reconfiguration

Run the script 

```
./run_batch_sim.sh 
```

