FROM python:3.10-slim

WORKDIR /app

# Install dependencies
RUN pip install --no-cache-dir \
    numpy \
    paho-mqtt

COPY . .

CMD ["python", "action_node.py"]
