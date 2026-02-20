#!/usr/bin/env python3
"""
Camera Host - Cattura frame dalla camera CSI del Jetson e li salva su disco.

Questo script gira SULL'HOST (fuori Docker) perché la camera CSI
richiede il driver NVIDIA Argus che non è disponibile nel container.

I frame vengono salvati nel volume condiviso con il container sense,
che li legge e li pubblica come topic ROS2.

NOTA: Usa GStreamer via subprocess (non OpenCV VideoCapture) perché
OpenCV 3.2 sul Jetson non ha il backend GStreamer compilato.

Uso:
    python3 camera_host.py

Il file viene salvato in: src/sense/shared_frame.jpg
(che nel container è: /home/ubuntu/sense_ws/shared_frame.jpg)
"""

import cv2
import numpy as np
import os
import time
import sys
import subprocess

# Configurazione
WIDTH = 640
HEIGHT = 480
FPS = 15
FRAME_SIZE = WIDTH * HEIGHT * 3  # BGR = 3 bytes per pixel

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_path = os.path.join(script_dir, 'shared_frame.jpg')
    tmp_path = os.path.join(script_dir, 'shared_frame.tmp.jpg')

    interval = 1.0 / FPS

    # Pipeline GStreamer che scrive frame BGR raw su stdout
    # Funziona perché gst-launch-1.0 supporta nvarguscamerasrc nativamente
    gst_cmd = [
        'gst-launch-1.0', '-q',
        'nvarguscamerasrc', '!',
        'video/x-raw(memory:NVMM), width={}, height={}, format=NV12, framerate={}/1'.format(WIDTH, HEIGHT, FPS), '!',
        'nvvidconv', 'flip-method=0', '!',
        'video/x-raw, width={}, height={}, format=BGRx'.format(WIDTH, HEIGHT), '!',
        'videoconvert', '!',
        'video/x-raw, format=BGR', '!',
        'fdsink', 'fd=1'
    ]

    print('[camera_host] Avvio GStreamer pipeline via subprocess...')
    print('[camera_host] Risoluzione: {}x{} @ {}fps'.format(WIDTH, HEIGHT, FPS))

    try:
        proc = subprocess.Popen(
            gst_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=FRAME_SIZE * 2
        )
    except FileNotFoundError:
        print('[camera_host] ERRORE: gst-launch-1.0 non trovato!')
        sys.exit(1)

    # Verifica che il processo sia partito
    time.sleep(1.0)
    if proc.poll() is not None:
        stderr = proc.stderr.read().decode('utf-8', errors='replace')
        print('[camera_host] ERRORE: GStreamer terminato subito!')
        print('[camera_host] Stderr: {}'.format(stderr))
        sys.exit(1)

    print('[camera_host] Pipeline attiva! Salvataggio in: {}'.format(output_path))
    print('[camera_host] Ctrl+C per uscire')

    frame_count = 0
    try:
        while True:
            t0 = time.time()

            # Leggi esattamente un frame BGR raw dallo stdout
            raw = proc.stdout.read(FRAME_SIZE)
            if len(raw) != FRAME_SIZE:
                print('[camera_host] WARN: frame incompleto ({}/{}), stream terminato?'.format(len(raw), FRAME_SIZE))
                break

            # Converti bytes raw in immagine numpy BGR
            frame = np.frombuffer(raw, dtype=np.uint8).reshape((HEIGHT, WIDTH, 3))

            # Salva come JPEG (scrittura atomica: tmp + rename)
            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                with open(tmp_path, 'wb') as f:
                    f.write(buf.tobytes())
                os.rename(tmp_path, output_path)
                frame_count += 1
                if frame_count % (FPS * 5) == 0:  # Log ogni 5 secondi
                    print('[camera_host] Frame #{} salvato ({}x{})'.format(frame_count, WIDTH, HEIGHT))

            # Mantieni il framerate
            elapsed = time.time() - t0
            if elapsed < interval:
                time.sleep(interval - elapsed)

    except KeyboardInterrupt:
        print('\n[camera_host] Chiusura...')
    finally:
        proc.terminate()
        proc.wait(timeout=5)
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
        print('[camera_host] Terminato dopo {} frame.'.format(frame_count))


if __name__ == '__main__':
    main()
