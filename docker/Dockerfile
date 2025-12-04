# Partiamo dall'immagine base con noVNC già configurato
FROM tiryoh/ros2-desktop-vnc:jazzy

# Rimaniamo come root. L'entrypoint dell'immagine base gestirà
# il passaggio all'utente 'ubuntu' all'avvio del container.
USER root

# Set environment
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Installiamo le dipendenze per la simulazione Gazebo e il bridge ROS2
RUN apt-get update && apt-get install -y \
    # Gazebo Sim integration
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    # Robot description tools
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    # ROS2 Control packages
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2controlcli \
    # Additional tools
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies for Charlie
RUN pip3 install --break-system-packages \
    py_trees \
    opencv-python-headless \
    numpy

# Setup workspace directory
WORKDIR /home/ubuntu/ros2_ws

# Source ROS2 automaticamente in .bashrc dell'utente ubuntu
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "source /home/ubuntu/ros2_ws/install/setup.bash 2>/dev/null || true" >> /home/ubuntu/.bashrc

# Fix ownership (l'entrypoint dell'immagine base se ne occuperà comunque)
RUN chown -R ubuntu:ubuntu /home/ubuntu

# Note: L'immagine tiryoh/ros2-desktop-vnc espone già:
# - Porta 80 per noVNC
# - Porta 5900 per VNC diretto
# E ha già configurato supervisord per gestire il desktop environment

# L'entrypoint dell'immagine base gestisce tutto automaticamente
