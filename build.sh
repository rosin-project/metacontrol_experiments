#!/bin/bash
set -e

sudo apt-get update
sudo apt-get install -y openjdk-11-jre python-catkin-tools
mkdir -p ~/metacontrol_ws/src
cd ~/metacontrol_ws
wstool init src https://raw.githubusercontent.com/rosin-project/metacontrol_experiments/master/metacontrol_experiments.rosinstall
rosdep update
rosdep install --from-paths ./src -y -i -r --skip-keys="abb_rws_interface"
source /opt/ros/melodic/setup.bash
catkin build --verbose