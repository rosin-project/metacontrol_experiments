# metacontrol_experiments
Controls simulation for metacontrol_sim

## Usage:

### Edit WS path variables
On the `run.sh` script edit the following path variables according to your workspaces

```
METACONTROL_WS_PATH="$HOME/ros/metacontrol_ws"
REASONER_WS_PATH="$HOME/ros/reasoner_metacontrol_ws"
PYTHON3_VENV_PATH="$HOME/ros/rospy3_melodic/"
```
### Run the script

Then just run

```
$ ./run.sh
```

### To run the `mros1_reasoner`

Uncomment lines 69 - 74
