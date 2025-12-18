#!/bin/bash

cd ~/ros2_ws/

colcon build

source install/setup.bash

ros2 launch irobot_create_gz_bringup create3_gz.launch.py