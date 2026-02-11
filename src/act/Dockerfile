FROM ros:foxy-ros-base

USER root

ENV ROS_DISTRO=foxy

# Copiamo il tuo codice sorgente dentro il container
WORKDIR /home/ubuntu/act_ws


# Installiamo dipendenze di sistema 
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Installiamo le dipendenze Python
COPY requirements.txt .
RUN pip3 install -r requirements.txt

# Aggiungi il source di ROS2 al .bashrc per le sessioni interattive
RUN echo "source /opt/ros/foxy/setup.bash" >> /home/ubuntu/.bashrc

# Copia entrypoint
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Passa all'utente ubuntu per l'esecuzione
USER ubuntu

ENTRYPOINT ["/entrypoint.sh"]

# Default command: avvia action_node
CMD ["python3", "action_node.py"]
