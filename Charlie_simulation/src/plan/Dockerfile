FROM ros:jazzy-ros-base

USER root

ENV ROS_DISTRO=jazzy

# Copiamo il tuo codice sorgente dentro il container
WORKDIR /home/ubuntu/plan_ws

# Installiamo dipendenze di sistema 
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Installiamo librerie Python necessarie
COPY requirements.txt .
RUN pip3 install -r requirements.txt --break-system-packages

# Aggiungi il source di ROS2 al .bashrc per le sessioni interattive
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/ubuntu/.bashrc

# Copia l'entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Passa all'utente ubuntu per l'esecuzione
USER ubuntu

ENTRYPOINT ["/entrypoint.sh"]

# Default command: avvia plan_node
CMD ["python3", "plan_node.py"]