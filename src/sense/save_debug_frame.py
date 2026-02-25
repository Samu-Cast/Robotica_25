#!/usr/bin/env python3
"""
Saves a frame from the camera and the debug image every 5 seconds.
Used for visual debugging and verification.
"""

import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
try:
    from image_utils import imgmsg_to_cv2
except ImportError:
    from .image_utils import imgmsg_to_cv2


class FrameSaver(Node):
    def __init__(self):
        super().__init__('frame_saver')

        self.last_raw = None
        self.last_debug = None

        self.create_subscription(Image, '/camera_front/image', self.raw_cb, 10)
        self.create_subscription(Image, '/sense/debug_image', self.debug_cb, 10)

        #Salva ogni 5 secondi
        self.create_timer(5.0, self.save_frames)
        self.get_logger().info('Frame saver attivo — salvataggio ogni 5 secondi (Ctrl+C per uscire)')

    def raw_cb(self, msg):
        self.last_raw = msg

    def debug_cb(self, msg):
        self.last_debug = msg

    def save_frames(self):
        ts = time.strftime('%H:%M:%S')
        if self.last_raw is not None:
            frame = imgmsg_to_cv2(self.last_raw, desired_encoding='bgr8')
            cv2.imwrite('/home/ubuntu/sense_ws/frame_raw.jpg', frame)
            self.get_logger().info(f'[{ts}] RAW salvato ({frame.shape})')
        else:
            self.get_logger().warn(f'[{ts}] Nessun frame RAW ricevuto')

        if self.last_debug is not None:
            frame = imgmsg_to_cv2(self.last_debug, desired_encoding='bgr8')
            cv2.imwrite('/home/ubuntu/sense_ws/frame_debug.jpg', frame)
            self.get_logger().info(f'[{ts}] DEBUG salvato ({frame.shape})')
        else:
            self.get_logger().warn(f'[{ts}] Nessun frame DEBUG ricevuto')


def main(args=None):
    rclpy.init(args=args)
    node = FrameSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
