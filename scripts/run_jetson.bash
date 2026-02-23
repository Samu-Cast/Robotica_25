#!/bin/bash
# =============================================================
# run_jetson.bash
# Script per avviare camera_host + docker compose sul Jetson
# Eseguire via SSH:  bash scripts/run_jetson.bash
# =============================================================

set -e

# ---- Percorsi ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CAMERA_SCRIPT="$PROJECT_DIR/src/sense/camera_host.py"
COMPOSE_DIR="$PROJECT_DIR/ros2_physical"

# ---- Colori ----
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# ---- Cleanup on exit ----
CAMERA_PID=""

cleanup() {
    echo ""
    echo -e "${YELLOW}[run_jetson] Arresto in corso...${NC}"

    # Ferma docker compose
    echo -e "${YELLOW}[run_jetson] Fermando docker compose...${NC}"
    docker-compose "$COMPOSE_DIR/docker-compose.yml" down 2>/dev/null || true

    # Ferma camera_host
    if [ -n "$CAMERA_PID" ] && kill -0 "$CAMERA_PID" 2>/dev/null; then
        echo -e "${YELLOW}[run_jetson] Fermando camera_host (PID $CAMERA_PID)...${NC}"
        kill "$CAMERA_PID" 2>/dev/null
        wait "$CAMERA_PID" 2>/dev/null
    fi

    # Pulisci file temporanei
    rm -f /dev/shm/shared_frame.jpg /dev/shm/shared_frame.tmp.jpg 2>/dev/null

    echo -e "${GREEN}[run_jetson] Tutto fermato. Bye!${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# ---- Controlli ----
if [ ! -f "$CAMERA_SCRIPT" ]; then
    echo -e "${RED}[run_jetson] ERRORE: $CAMERA_SCRIPT non trovato!${NC}"
    exit 1
fi

if [ ! -f "$COMPOSE_DIR/docker-compose.yml" ]; then
    echo -e "${RED}[run_jetson] ERRORE: docker-compose.yml non trovato in $COMPOSE_DIR!${NC}"
    exit 1
fi

# ---- 1. Avvia camera_host in background ----
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  AVVIO CAMERA HOST                     ${NC}"
echo -e "${GREEN}========================================${NC}"

python3 "$CAMERA_SCRIPT" &
CAMERA_PID=$!
echo -e "${GREEN}[run_jetson] camera_host avviato (PID $CAMERA_PID)${NC}"

# Aspetta che la camera sia pronta (primo frame salvato)
echo -e "${YELLOW}[run_jetson] Aspetto che la camera sia pronta...${NC}"
TIMEOUT=10
ELAPSED=0
while [ ! -f /dev/shm/shared_frame.jpg ] && [ $ELAPSED -lt $TIMEOUT ]; do
    sleep 1
    ELAPSED=$((ELAPSED + 1))

    # Controlla che camera_host sia ancora vivo
    if ! kill -0 "$CAMERA_PID" 2>/dev/null; then
        echo -e "${RED}[run_jetson] ERRORE: camera_host terminato prematuramente!${NC}"
        exit 1
    fi
done

if [ -f /dev/shm/shared_frame.jpg ]; then
    echo -e "${GREEN}[run_jetson] Camera pronta! Primo frame salvato.${NC}"
else
    echo -e "${YELLOW}[run_jetson] WARN: Timeout aspettando il primo frame, proseguo comunque...${NC}"
fi

# ---- 2. Avvia docker compose con log in tempo reale ----
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  AVVIO DOCKER COMPOSE                  ${NC}"
echo -e "${GREEN}========================================${NC}"

echo -e "${YELLOW}[run_jetson] Building e avvio containers...${NC}"
echo -e "${YELLOW}[run_jetson] Premi Ctrl+C per fermare tutto${NC}"
echo ""

# docker compose up con log in tempo reale (foreground)
docker-compose "$COMPOSE_DIR/docker-compose.yml" up
