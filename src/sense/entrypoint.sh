#!/bin/bash
set -e

#Carica l'ambiente ROS 2 Foxy
source /opt/ros/foxy/setup.bash

#Carica il workspace irobot_create_msgs
if [ -f /home/ubuntu/irobot_ws/install/setup.bash ]; then
  source /home/ubuntu/irobot_ws/install/setup.bash
fi

#Carica il workspace del modulo (se compilato)
if [ -f /home/ubuntu/sense_ws/install/setup.bash ]; then
  source /home/ubuntu/sense_ws/install/setup.bash
fi

exec "$@"