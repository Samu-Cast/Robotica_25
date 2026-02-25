#!/usr/bin/env python3
"""
Camera Node - Reads frames from the shared volume and publishes to ROS2.

Publishes:
    - /camera_front/image (Image): BGR frames from the camera
"""

import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from .image_utils import cv2_to_imgmsg
except ImportError:
    from image_utils import cv2_to_imgmsg


SHARED_FRAME_PATH = '/dev/shm/shared_frame.jpg'


class CameraNode(Node):
    """ROS2 node that reads images from disk and republishes them as a topic."""

    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info('=== Camera Node Starting (shared volume mode) ===')

        self.pub = self.create_publisher(Image, '/camera_front/image', 10)
        self.frame_path = SHARED_FRAME_PATH
        self.last_mtime = 0.0

        self.get_logger().info(f'Leggo frame da: {self.frame_path}')
        self.get_logger().info('Assicurati che camera_host.py sia in esecuzione sull\'host!')

        #15 FPS timer to check for and publish new frames
        self.timer = self.create_timer(1.0 / 15, self._publish_frame)

        #Log periodico se il file non esiste
        self._warn_count = 0

    def _publish_frame(self):
        if not os.path.exists(self.frame_path):
            return

        try:
            mtime = os.path.getmtime(self.frame_path)
            if mtime <= self.last_mtime:
                return 

            self.last_mtime = mtime
            
            #Lettura del frame
            frame = cv2.imread(self.frame_path)

            if frame is not None:
                msg = cv2_to_imgmsg(frame, encoding='bgr8')
                #Usa il tempo di modifica del file per una sincronizzazione più precisa
                #invece del tempo "attuale" di ricezione
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_front_link' #Convenzione standard _link
                self.pub.publish(msg)
            else:
                self.get_logger().warn('Frame letto corrotto o vuoto', throttle_duration_sec=5.0)

        except Exception as e:
            self.get_logger().error(f'Errore critico: {e}', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
