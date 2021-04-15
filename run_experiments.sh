#!/bin/bash
set -e

source ~/metacontrol_ws/devel/setup.bash
roscd metacontrol_experiments
./run_bash_sim.sh $1
