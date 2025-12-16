#!/bin/bash
set -e

# Carica l'ambiente ROS 2 base
source /opt/ros/jazzy/setup.bash

# Carica il tuo workspace (se compilato)
if [ -f /home/ubuntu/act_ws/install/setup.bash ]; then
  source /home/ubuntu/act_ws/install/setup.bash
fi

# Esegue il comando passato nel CMD del Dockerfile
exec "$@"