FROM python:3.10-slim

WORKDIR /app

# Install Python dependencies
RUN pip3 install -r ./requirements.txt --break-system-packages

COPY . .

CMD ["python", "controller_node.py"]
