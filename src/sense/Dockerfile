# Partiamo dall'immagine ufficiale ROS 2 Jazzy (versione base, no GUI)
FROM ros:jazzy-ros-base

# Rimaniamo come root. L'entrypoint dell'immagine base gestirà
# il passaggio all'utente 'ubuntu' all'avvio del container.
USER root

# Set environment
ENV ROS_DISTRO=jazzy
ENV PYTHONUNBUFFERED=1

WORKDIR /home/ubuntu/sense_ws

# Installiamo dipendenze di sistema 
RUN apt-get update && apt-get install -y \
    python3-pip \
    libglib2.0-0 \
    libgl1 \
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-opencv \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Installiamo YOLO e librerie Python necessarie
COPY requirements.txt .
RUN pip3 install -r requirements.txt --break-system-packages 

# Copia entrypoint
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

# Default command: avvia sense_node
CMD ["python3", "sense_node.py"]