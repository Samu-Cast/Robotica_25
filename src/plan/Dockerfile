FROM python:3.10-slim

WORKDIR /app

# Install dependencies
RUN pip install --no-cache-dir \
    py_trees \
    opencv-python-headless \
    numpy \
    paho-mqtt

COPY . .

CMD ["python", "controller_node.py"]
