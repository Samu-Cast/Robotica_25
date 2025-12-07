FROM python:3.10-slim

WORKDIR /app

# Install camera/sensor dependencies
RUN pip install --no-cache-dir \
    opencv-python-headless \
    numpy \
    paho-mqtt

COPY . .

CMD ["python", "sense_node.py"]
