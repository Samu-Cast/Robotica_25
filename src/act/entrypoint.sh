#!/bin/bash
set -e

#Carica l'ambiente ROS 2 base
source /opt/ros/foxy/setup.bash

if [ -f /home/ubuntu/act_ws/install/setup.bash ]; then
  source /home/ubuntu/act_ws/install/setup.bash
fi

exec "$@"