FROM ros:jazzy-ros-base

USER root

ENV ROS_DISTRO=jazzy

# Installiamo dipendenze di sistema 
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Copiamo il tuo codice sorgente dentro il container
WORKDIR /home/ubuntu/act_ws

#ENTRYPOINT [ "./entrypoint.sh" ]


# Avvio del nodo ACT
#CMD source /opt/ros/jazzy/setup.bash && python3 action_node.py
