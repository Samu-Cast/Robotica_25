#Modulo Plan – ROS 2 Foxy
FROM ros:foxy-ros-base

USER root

#Variabili d'ambiente
ENV ROS_DISTRO=foxy
ENV PYTHONUNBUFFERED=1
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /home/ubuntu/plan_ws

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
    https://github.com/iRobotEducation/irobot_create_msgs.git

# Aggiungi i .msg audio mancanti nel branch foxy (esistono da galactic in poi)
RUN printf '%s\n' \
    "uint16 frequency" \
    "builtin_interfaces/Duration max_runtime" \
    > /home/ubuntu/irobot_ws/src/irobot_create_msgs/msg/AudioNote.msg && \
    printf '%s\n' \
    "std_msgs/Header header" \
    "irobot_create_msgs/AudioNote[] notes" \
    "bool append" \
    > /home/ubuntu/irobot_ws/src/irobot_create_msgs/msg/AudioNoteVector.msg && \
    sed -i '/"msg\/WheelVels.msg"/a\  "msg/AudioNote.msg"\n  "msg/AudioNoteVector.msg"' \
    /home/ubuntu/irobot_ws/src/irobot_create_msgs/CMakeLists.txt

WORKDIR /home/ubuntu/irobot_ws
RUN . /opt/ros/foxy/setup.sh && \
    colcon build --symlink-install --packages-select irobot_create_msgs

#Dipendenze Python (py_trees)
WORKDIR /home/ubuntu/plan_ws
COPY requirements.txt .
RUN pip3 install -r requirements.txt

#Configurazione shell
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc && \
    echo "source /home/ubuntu/irobot_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc

#Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

#Default command: avvia plan_node
CMD ["python3", "plan_node.py"]
