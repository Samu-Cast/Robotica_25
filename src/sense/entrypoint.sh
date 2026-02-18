#!/bin/bash
set -e

#Rimuovi opencv-python di pip (se presente) per evitare conflitti con cv_bridge
pip3 uninstall -y opencv-python-headless opencv-python 2>/dev/null || true

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

#Avvia camera_node in background (pubblica /camera_front/image dalla CSI)
echo "[entrypoint] Avvio camera_node.py in background..."
python3 /home/ubuntu/sense_ws/camera_node.py &
CAMERA_PID=$!
sleep 2  # Attendi che la camera CSI si inizializzi

#Pulisci il processo camera all'uscita del container
trap "kill $CAMERA_PID 2>/dev/null; wait $CAMERA_PID 2>/dev/null" EXIT

echo "[entrypoint] Camera avviata (PID=$CAMERA_PID), avvio comando principale..."
exec "$@"