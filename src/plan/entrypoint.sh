#!/bin/bash
set -e

# Carica l'ambiente ROS 2 Foxy
source /opt/ros/foxy/setup.bash

# Carica il workspace irobot_create_msgs
if [ -f /home/ubuntu/irobot_ws/install/setup.bash ]; then
  source /home/ubuntu/irobot_ws/install/setup.bash
fi

# Carica il workspace del modulo (se compilato)
if [ -f /home/ubuntu/plan_ws/install/setup.bash ]; then
  source /home/ubuntu/plan_ws/install/setup.bash
fi

# Esegue il comando passato nel CMD del Dockerfile
exec "$@"
