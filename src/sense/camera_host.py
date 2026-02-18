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
    """Prova ad aprire la camera con diversi metodi."""

    # 1) V4L2 diretto
    print('[camera_host] Tentativo 1: V4L2 diretto...')
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        print('[camera_host] Camera aperta via V4L2')
        return cap

    # 2) Auto-backend
    print('[camera_host] Tentativo 2: auto-backend...')
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        print('[camera_host] Camera aperta via auto-backend')
        return cap

    # 3) GStreamer (nvarguscamerasrc per camera CSI Jetson)
    pipeline = (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), width=640, height=480, framerate=15/1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink"
    )
    print(f'[camera_host] Tentativo 3: GStreamer nvarguscamerasrc...')
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if cap.isOpened():
        print('[camera_host] Camera aperta via GStreamer nvarguscamerasrc')
        return cap

    # 4) GStreamer v4l2src
    pipeline = (
        "v4l2src device=/dev/video0 ! "
        "video/x-raw, width=640, height=480 ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink"
    )
    print(f'[camera_host] Tentativo 4: GStreamer v4l2src...')
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if cap.isOpened():
        print('[camera_host] Camera aperta via GStreamer v4l2src')
        return cap

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
