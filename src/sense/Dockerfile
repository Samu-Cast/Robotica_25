#Immagine base: ROS 2 Foxy
FROM ros:foxy-ros-base

USER root

ENV ROS_DISTRO=foxy
ENV PYTHONUNBUFFERED=1

WORKDIR /home/ubuntu/sense_ws

RUN apt-get update && apt-get install -y \
    python3-pip \
    libglib2.0-0 \
    libgl1 \
    ros-foxy-cv-bridge \
    ros-foxy-vision-opencv \
    sudo \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip3 install --no-cache-dir --default-timeout=300 --retries 10 -r requirements.txt
RUN echo "source /opt/ros/foxy/setup.bash" >> /home/ubuntu/.bashrc

COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ubuntu

ENTRYPOINT ["/entrypoint.sh"]

CMD ["python3", "sense_node.py"]