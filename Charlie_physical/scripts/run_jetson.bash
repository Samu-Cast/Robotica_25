#!/bin/bash

#script per avviare il robot fisico dentro il jetson nano

cd ../src/sense
python3 camera_host.py &

sleep 3

cd ../docker
docker-compose up
