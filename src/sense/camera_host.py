#!/usr/bin/env python3
"""
Camera Host - Cattura frame dalla camera CSI del Jetson e li salva su disco.

Questo script gira SULL'HOST (fuori Docker) perché la camera CSI
richiede il driver NVIDIA Argus che non è disponibile nel container.

I frame vengono salvati nel volume condiviso con il container sense,
che li legge e li pubblica come topic ROS2.

Uso:
    python3 camera_host.py

Il file viene salvato in: src/sense/shared_frame.jpg
(che nel container è: /home/ubuntu/sense_ws/shared_frame.jpg)
"""

import cv2
import os
import time
import sys

def open_camera():
    """Ottimizzato per Jetson CSI Camera."""
    
    # === 1. Pipeline GStreamer con nvarguscamerasrc (Jetson CSI) ===
    pipeline = (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw, width=640, height=480, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! appsink drop=True"
    )

    print('[camera_host] Tentativo 1: GStreamer nvarguscamerasrc...')
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    
    if cap.isOpened():
        print('[camera_host] Camera CSI aperta con successo!')
        return cap
    print('[camera_host] nvarguscamerasrc fallito.')

    # === 2. Pipeline GStreamer con v4l2src (USB camera o /dev/video0) ===
    pipeline_v4l2 = (
        "v4l2src device=/dev/video0 ! "
        "video/x-raw, width=640, height=480, framerate=30/1 ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! appsink drop=True"
    )
    print('[camera_host] Tentativo 2: GStreamer v4l2src...')
    cap = cv2.VideoCapture(pipeline_v4l2, cv2.CAP_GSTREAMER)
    
    if cap.isOpened():
        print('[camera_host] Camera V4L2 (GStreamer) aperta!')
        return cap
    print('[camera_host] v4l2src fallito.')

    # === 3. Fallback: OpenCV diretto (compatibile con vecchie versioni) ===
    print('[camera_host] Tentativo 3: OpenCV diretto /dev/video0...')
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        print('[camera_host] Camera aperta con OpenCV diretto!')
        return cap

    # Diagnostica
    print('[camera_host] TUTTI I TENTATIVI FALLITI!')
    print('[camera_host] Diagnostica:')
    print(f'[camera_host]   OpenCV version: {cv2.__version__}')
    print(f'[camera_host]   GStreamer support: {cv2.getBuildInformation().find("GStreamer") > 0}')
    import subprocess
    result = subprocess.run(['ls', '-la', '/dev/video*'], capture_output=True, text=True, shell=False)
    print(f'[camera_host]   /dev/video*: {result.stdout.strip() or "NESSUN DEVICE TROVATO"}')
    return None

def main():
    # Percorso di output: nella stessa cartella dello script (volume condiviso)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_path = os.path.join(script_dir, 'shared_frame.jpg')
    tmp_path = os.path.join(script_dir, 'shared_frame.tmp.jpg')

    fps = 15
    interval = 1.0 / fps

    cap = open_camera()
    if cap is None:
        print('[camera_host] ERRORE: impossibile aprire la camera!')
        print('[camera_host] Verifica che /dev/video0 esista e la camera sia collegata.')
        sys.exit(1)

    print(f'[camera_host] Cattura attiva a ~{fps} FPS')
    print(f'[camera_host] Salvataggio in: {output_path}')
    print('[camera_host] Ctrl+C per uscire')

    frame_count = 0
    try:
        while True:
            t0 = time.time()
            ret, frame = cap.read()
            if ret:
                # Scrittura atomica: scrivi su .tmp poi rinomina
                cv2.imwrite(tmp_path, frame)
                os.rename(tmp_path, output_path)
                frame_count += 1
                if frame_count % (fps * 5) == 0:  # Log ogni 5 secondi
                    print(f'[camera_host] Frame #{frame_count} salvato ({frame.shape})')
            else:
                print('[camera_host] WARN: frame non letto, riprovo...')
                time.sleep(0.5)

            # Mantieni il framerate
            elapsed = time.time() - t0
            if elapsed < interval:
                time.sleep(interval - elapsed)

    except KeyboardInterrupt:
        print('\n[camera_host] Chiusura...')
    finally:
        cap.release()
        # Pulizia file temporaneo
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
        print('[camera_host] Camera rilasciata.')


if __name__ == '__main__':
    main()
