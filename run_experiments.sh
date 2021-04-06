#!/bin/bash
set -e

source ~/metacontrol_ws/devel/setup.bash
roscd metacontrol_experiments
./run_single_sim_desktop.sh -i 1 -g 1 -n "f1_v2_r2" -o 2 -r "true" -p 1.7 -c "false"
