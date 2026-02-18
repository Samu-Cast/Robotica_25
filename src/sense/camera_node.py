#!/usr/bin/env python3
"""
Camera Node - Cattura frame dalla camera e li pubblica su ROS2

Pubblica:
    - /camera_front/image (Image) - frame BGR dalla camera

Prova ad aprire la camera via V4L2 diretto (/dev/video0),
con fallback a pipeline GStreamer se necessario.
"""

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from .image_utils import cv2_to_imgmsg
except ImportError:
    from image_utils import cv2_to_imgmsg


class CameraNode(Node):
    """Nodo ROS2 che cattura frame dalla camera e li pubblica."""

    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info('=== Camera Node Starting ===')

        self.pub = self.create_publisher(Image, '/camera_front/image', 10)

        # Prova ad aprire la camera: prima V4L2 diretto, poi GStreamer come fallback
        self.cap = None
        self._open_camera()

        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error(
                'Impossibile aprire la camera! '
                'Verifica: 1) device montato (--device /dev/video0) '
                '2) permessi corretti su /dev/video0'
            )
            return

        self.get_logger().info('Camera aperta con successo')
        # Timer a 15 FPS per catturare e pubblicare frame
        self.timer = self.create_timer(1.0 / 15, self._publish_frame)

    def _open_camera(self):
        """Prova ad aprire la camera con diversi backend, in ordine di priorità."""

        # 1) V4L2 diretto — il più affidabile dentro Docker
        self.get_logger().info('Tentativo 1: V4L2 diretto /dev/video0 ...')
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.get_logger().info('Camera aperta via V4L2 diretto')
            self.cap = cap
            return

        # 2) Device index generico (OpenCV sceglie il backend)
        self.get_logger().info('Tentativo 2: device index 0 (auto-backend) ...')
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.get_logger().info('Camera aperta via auto-backend')
            self.cap = cap
            return

        # 3) GStreamer pipeline come ultimo tentativo
        pipeline = (
            "v4l2src device=/dev/video0 ! "
            "video/x-raw, width=640, height=480 ! "
            "videoconvert ! video/x-raw, format=BGR ! appsink"
        )
        self.get_logger().info(f'Tentativo 3: GStreamer pipeline: {pipeline}')
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            self.get_logger().info('Camera aperta via GStreamer')
            self.cap = cap
            return

        self.get_logger().error('Tutti i tentativi di apertura camera falliti!')

    def _publish_frame(self):
        """Cattura un frame dalla camera e lo pubblica sul topic."""
        ret, frame = self.cap.read()
        if ret:
            msg = cv2_to_imgmsg(frame, encoding='bgr8')
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
