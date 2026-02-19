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

#Avvia camera_node in background (legge frame da /dev/shm/shared_frame.jpg e pubblica su /camera_front/image)
#NOTA: camera_host.py deve essere avviato sull'host Jetson per catturare i frame
echo "[entrypoint] Avvio camera_node.py in background (shared volume mode)..."
python3 /home/ubuntu/sense_ws/camera_node.py &
CAMERA_PID=$!

#Pulisci il processo camera all'uscita del container
trap "kill $CAMERA_PID 2>/dev/null; wait $CAMERA_PID 2>/dev/null" EXIT

echo "[entrypoint] Camera reader avviato (PID=$CAMERA_PID), avvio comando principale..."
exec "$@"