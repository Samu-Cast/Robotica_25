ROS2 Wrappers - FASE sviluppo virtuale

Questi file wrappano il codice FASE 1 in nodi ROS2 per deployment su Gazebo.

File:
- perception_node.py: ROS2 wrapper per YOLO detector
- navigation_node.py: ROS2 navigation node
- detection.py: Legacy OpenCV detector (backup)
- launch/: ROS2 launch files
- setup.py, package.xml: ROS2 package configuration

Uso (FASE sviluppo virtuale):
1. Build: colcon build --symlink-install
2. Launch: ros2 launch charlie charlie.launch.py
