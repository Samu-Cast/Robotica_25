#Immagine base: ROS 2 Foxy
FROM ros:foxy-ros-base

USER root

ENV ROS_DISTRO=foxy

WORKDIR /home/ubuntu/plan_ws

RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip3 install -r requirements.txt

RUN echo "source /opt/ros/foxy/setup.bash" >> /home/ubuntu/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ubuntu

ENTRYPOINT ["/entrypoint.sh"]
CMD ["python3", "plan_node.py"]
