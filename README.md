# metacontrol_experiments
Controls simulation for metacontrol_sim

## Usage:

### Edit WS path variables
create a `config.sh` file in the same folder as the `run.sh` defining your values for these variables

```
#!/bin/bash

METACONTROL_WS_PATH=path/to/your/metacontrol_ws
REASONER_WS_PATH=path/to/your/metacontrol_ws
PYTHON3_VENV_PATH=path/to/your/metacontrol_ws
```

### Run the script

Then just run

```
$ ./run.sh
```

### To run the `mros1_reasoner`

Uncomment lines 69 - 74
