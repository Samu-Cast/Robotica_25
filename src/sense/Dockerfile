#Modulo Sense – ROS 2 Foxy
FROM ros:foxy-ros-base

USER root

#Variabili d'ambiente
ENV ROS_DISTRO=foxy
ENV PYTHONUNBUFFERED=1
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /home/ubuntu/sense_ws

#Dipendenze di sistema
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    #Vision / OpenCV bridge
    ros-foxy-cv-bridge \
    ros-foxy-vision-opencv \
    #Librerie di sistema per OpenCV
    libglib2.0-0 \
    libgl1-mesa-glx \
    #CycloneDDS per compatibilità iCreate3
    ros-foxy-rmw-cyclonedds-cpp \
    #Utilità
    git \
    sudo \
    && rm -rf /var/lib/apt/lists/*

#irobot_create_msgs (build da sorgente per Foxy)
WORKDIR /home/ubuntu/irobot_ws/src
RUN git clone -b foxy \
    https://github.com/iRobotEducation/irobot_create_msgs.git
WORKDIR /home/ubuntu/irobot_ws
RUN . /opt/ros/foxy/setup.sh && \
    colcon build --symlink-install --packages-select irobot_create_msgs

#Dipendenze Python (YOLO, OpenCV, NumPy)
WORKDIR /home/ubuntu/sense_ws
COPY requirements.txt .
RUN pip3 install --no-cache-dir --default-timeout=300 --retries 10 \
    -r requirements.txt --break-system-packages || \
    pip3 install --no-cache-dir --default-timeout=300 --retries 10 \
    -r requirements.txt

#Configurazione shell
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc && \
    echo "source /home/ubuntu/irobot_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc

#Entrypoint
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

#Default command: avvia sense_node
CMD ["python3", "sense_node.py"]
