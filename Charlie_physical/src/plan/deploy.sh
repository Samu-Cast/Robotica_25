#!/bin/bash

docker stop plan_module 2>/dev/null
docker rm plan_module 2>/dev/null
docker rmi plan_image 2>/dev/null

docker build -t plan_image .

docker run -d --name plan_module \
  --network ros_network \
  plan_image

docker ps

echo "Plan module deployed"
