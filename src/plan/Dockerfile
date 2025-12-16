FROM ros:jazzy-ros-base

LABEL maintainer="Charlie Team"
LABEL description="Plan Module - Minimal BT Decision Making"

# Install Python dependencies
RUN pip3 install -r ./requirements.txt --break-system-packages

# Copy source
WORKDIR /ros2_ws
COPY plan_node.py .

# Run
CMD ["python3", "plan_node.py"]
