FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /app

# Dipendenze Python extra (se servono)
RUN pip install --no-cache-dir numpy paho-mqtt

# Copia il nodo ACT
COPY action_node.py .

# Installiamo le dipendenze Python
RUN pip3 install -r ./requirements.txt --break-system-packages

# Copiamo il tuo codice sorgente dentro il container
WORKDIR /home/ubuntu/act_ws

#ENTRYPOINT [ "./entrypoint.sh" ]


# Avvio del nodo ACT
#CMD source /opt/ros/jazzy/setup.bash && python3 action_node.py
