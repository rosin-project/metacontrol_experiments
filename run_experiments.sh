#!/bin/bash
set -e

source ~/metacontrol_ws/devel/setup.bash
roscd metacontrol_experiments
./run_batch_sim.sh $1 $2 $3 $4 $5
