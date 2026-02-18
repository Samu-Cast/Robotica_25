#!/usr/bin/env python3
"""
Sostituto leggero di cv_bridge usando solo numpy.
Evita la dipendenza da cv_bridge (che richiede compatibilità ABI
tra OpenCV di sistema e numpy di pip).
"""

import numpy as np
from sensor_msgs.msg import Image


def imgmsg_to_cv2(msg, desired_encoding='bgr8'):
    """Converte un messaggio ROS Image in un array numpy (come cv_bridge).
    
    Args:
        msg: sensor_msgs/Image message
        desired_encoding: ignorato se msg.encoding corrisponde già
        
    Returns:
        numpy array (H, W, 3) uint8 per immagini BGR/RGB
    """
    channels = 3
    dtype = np.uint8

    if msg.encoding in ('mono8',):
        channels = 1
    elif msg.encoding in ('bgr8', 'rgb8'):
        channels = 3
    elif msg.encoding in ('bgra8', 'rgba8'):
        channels = 4
    elif msg.encoding in ('16UC1', 'mono16'):
        channels = 1
        dtype = np.uint16

    img = np.frombuffer(msg.data, dtype=dtype)
    img = img.reshape(msg.height, msg.width, channels) if channels > 1 else img.reshape(msg.height, msg.width)

    # Conversione encoding se necessario
    if desired_encoding == 'bgr8' and msg.encoding == 'rgb8':
        img = img[:, :, ::-1].copy()
    elif desired_encoding == 'rgb8' and msg.encoding == 'bgr8':
        img = img[:, :, ::-1].copy()

    return img


def cv2_to_imgmsg(frame, encoding='bgr8'):
    """Converte un array numpy OpenCV in un messaggio ROS Image.
    
    Args:
        frame: numpy array (H, W, 3) o (H, W) uint8
        encoding: stringa encoding ('bgr8', 'rgb8', 'mono8')
        
    Returns:
        sensor_msgs/Image message
    """
    msg = Image()
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.encoding = encoding

    if len(frame.shape) == 3:
        msg.step = frame.shape[1] * frame.shape[2]
    else:
        msg.step = frame.shape[1]

    msg.data = frame.tobytes()
    return msg
