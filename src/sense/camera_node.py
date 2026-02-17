#!/usr/bin/env python3
"""
Camera Node - Cattura frame dalla camera CSI del Jetson e li pubblica su ROS2

Pubblica:
    - /camera_front/image (Image) - frame BGR dalla camera CSI

Utilizza la pipeline GStreamer nvarguscamerasrc per accedere alla camera CSI
(es. IMX219) attraverso l'interfaccia NVIDIA Argus del Jetson.
"""

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def get_jetson_gstreamer_source(width=640, height=480, fps=30):
    """Pipeline GStreamer V4L2 per camera su Jetson via /dev/video0.

    Usa v4l2src invece di nvarguscamerasrc, così funziona dentro Docker
    senza bisogno di runtime nvidia o plugin L4T.
    """
    return (
        f"v4l2src device=/dev/video0 ! "
        f"video/x-raw, width={width}, height={height}, framerate={fps}/1 ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! appsink"
    )


class CameraNode(Node):
    """Nodo ROS2 che cattura frame dalla camera CSI e li pubblica."""

    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info('=== Camera Node Starting ===')

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera_front/image', 10)

        # Apri camera CSI via GStreamer
        pipeline = get_jetson_gstreamer_source()
        self.get_logger().info(f'GStreamer pipeline: {pipeline}')
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error(
                'Impossibile aprire la camera CSI! '
                'Verifica: 1) device montato (--device /dev/video0) '
                '2) argus_socket montato (/tmp/argus_socket) '
                '3) runtime nvidia abilitato'
            )
            return

        self.get_logger().info('Camera CSI aperta con successo')
        # Timer a 30 FPS per catturare e pubblicare frame
        self.timer = self.create_timer(1.0 / 30, self._publish_frame)

    def _publish_frame(self):
        """Cattura un frame dalla camera e lo pubblica sul topic."""
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_front'
            self.pub.publish(msg)
        else:
            self.get_logger().warn('Frame non letto dalla camera', throttle_duration_sec=5.0)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        super().destroy_node()


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
