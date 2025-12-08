FROM python:3.10-slim

WORKDIR /app

# Install OpenCV and YOLO dependencies
RUN pip install --no-cache-dir \
    opencv-python-headless \
    ultralytics \
    numpy \
    paho-mqtt

COPY . .

CMD ["python", "perception_node.py"]
