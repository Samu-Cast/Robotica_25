#!/bin/bash


cd ../src/sense
python3 camera_host.py &

sleep 3

cd ../ros2_physical
docker-compose up
