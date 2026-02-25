#!/bin/bash
# Deploy script - Build e run del modulo Plan

# Stop e rimuovi vecchio container/image
docker stop plan_module 2>/dev/null
docker rm plan_module 2>/dev/null
docker rmi plan_image 2>/dev/null

# Build nuova immagine
docker build -t plan_image .

# Run container
docker run -d --name plan_module \
  --network ros_network \
  plan_image

# Mostra containers attivi
docker ps

echo "✅ Plan module deployed"
