FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /app

# Dipendenze Python extra (se servono)
RUN pip install --no-cache-dir numpy paho-mqtt

# Copia il nodo ACT
COPY action_node.py .

# Avvio del nodo ACT
CMD source /opt/ros/jazzy/setup.bash && python3 action_node.py
