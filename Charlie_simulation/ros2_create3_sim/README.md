# PER UNA BUILD PULITA
# eseguire i passi:

1. una volta dentro il container docker eseguire: 
    cd ~/ros2_ws/

2. (se presenti le cartelle create da colcon) 
    rm -rf build/ install log/

3. colcon build --symlink-install --executor sequential

4. source install/setup.bash

5. ros2 launch irobot_create_gz_bringup create3_gz.launch.py

