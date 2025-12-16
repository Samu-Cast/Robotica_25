FROM python:3.10-slim

WORKDIR /app

# Install dependencies
RUN pip install --no-cache-dir \
    numpy \
    paho-mqtt

COPY . .

# Installiamo le dipendenze Python
RUN pip3 install -r ./requirements.txt --break-system-packages

# Copiamo il tuo codice sorgente dentro il container
WORKDIR /home/ubuntu/act_ws

#ENTRYPOINT [ "./entrypoint.sh" ]


# Avvio del nodo ACT
#CMD source /opt/ros/jazzy/setup.bash && python3 action_node.py
