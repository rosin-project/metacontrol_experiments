# metacontrol_experiments
Controls simulation for metacontrol_sim

## Usage:

### Edit WS path variables
create a `config.sh` file in the same folder as the `run.sh` defining your values for these variables

export METACONTROL_WS_PATH
export REASONER_WS_PATH
export PYTHON3_VENV_PATH

### Run the script

Then just run

```
$ ./run.sh
```

### To run the `mros1_reasoner`

Uncomment lines 69 - 74
