#Modulo Act – ROS 2 Foxy
FROM ros:foxy-ros-base

USER root

#Variabili d'ambiente
ENV ROS_DISTRO=foxy
ENV PYTHONUNBUFFERED=1
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /home/ubuntu/act_ws

#Dipendenze di sistema
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    #CycloneDDS per compatibilità iCreate3
    ros-foxy-rmw-cyclonedds-cpp \
    #Utilità
    git \
    && rm -rf /var/lib/apt/lists/*

#irobot_create_msgs (build da sorgente per Foxy)
WORKDIR /home/ubuntu/irobot_ws/src
RUN git clone -b foxy \
    https://github.com/iRobot/irobot_create_msgs.git
WORKDIR /home/ubuntu/irobot_ws
RUN . /opt/ros/foxy/setup.sh && \
    colcon build --symlink-install --packages-select irobot_create_msgs

#Dipendenze Python
WORKDIR /home/ubuntu/act_ws
COPY requirements.txt .
RUN pip3 install -r requirements.txt --break-system-packages || \
    pip3 install -r requirements.txt

#Configurazione shell
RUN echo "source /opt/ros/foxy/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "source /home/ubuntu/irobot_ws/install/setup.bash 2>/dev/null || true" >> /home/ubuntu/.bashrc

#Entrypoint
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

#Utente non-root
USER ubuntu

ENTRYPOINT ["/entrypoint.sh"]

#Default command: avvia action_node
CMD ["python3", "action_node.py"]
