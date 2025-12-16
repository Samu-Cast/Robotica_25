#!/bin/bash
# Drop script - Ferma e rimuove container e immagine Plan

docker stop plan_module
docker rm plan_module
docker rmi plan_image

echo "✅ Plan module dropped"
