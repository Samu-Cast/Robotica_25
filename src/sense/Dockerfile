FROM python:3.10-slim

WORKDIR /app

# Set environment
ENV ROS_DISTRO=jazzy

# Installiamo dipendenze di sistema 
RUN apt-get update && apt-get install -y \
    python3-pip \
    libglib2.0-0 \
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-opencv \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Installiamo YOLO e librerie Python necessarie
# (Ultralytics include già numpy, torch, ecc.)
RUN pip3 install -r ./requirements.txt --break-system-packages 

WORKDIR /home/ubuntu/sense_ws

# Compiliamo il workspace (per rendere visibili i tuoi nodi custom)
#RUN . /opt/ros/jazzy/setup.sh && colcon build

COPY . .

CMD ["python", "sense_node.py"]
