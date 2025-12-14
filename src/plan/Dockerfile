FROM ros:jazzy-ros-base

LABEL maintainer="Charlie Team"
LABEL description="Plan Node - Behavior Tree Decision Making"

WORKDIR /ros2_ws

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --break-system-packages \
    py_trees \
    numpy

# Copy source code
COPY plan_node.py /ros2_ws/src/plan/
COPY behavior_tree.py /ros2_ws/src/plan/

# Entry point
CMD ["python3", "src/plan/plan_node.py"]
