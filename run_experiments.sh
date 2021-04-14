#!/bin/bash
set -e

source ~/metacontrol_ws/devel/setup.bash
roscd metacontrol_experiments
./run_single_sim.sh -i 1 -g 1 -n $1 -o 2 -r "true" -p 1.7 -c "false" -l 1.0
