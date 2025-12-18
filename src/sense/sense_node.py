#!/usr/bin/env python3
"""
Sense Node - Aggregates all sensor data and publishes to /sense/world_state

Subscribes to:
    - /ultrasonic_front/scan, /ultrasonic_front_left/scan, /ultrasonic_front_right/scan (LaserScan)
    - /camera_front/image (Image) for color detection and human detection
    - /odom (Odometry) for robot position
    - /battery_state (BatteryState) for battery level

Publishes:
    - /sense/world_state (String) - JSON with all aggregated data

Uses modules:
    - color_detector.py for colored target detection
    - human_detector.py for person detection (YOLO)
"""

import json
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge

# Import detection modules (try relative first, then absolute for direct execution)
try:
    from .color_detector import ColorDetector
    from .human_detector import HumanDetector
except ImportError:
    from color_detector import ColorDetector
    from human_detector import HumanDetector


class SenseNode(Node):
    """
    Main Sense Node - Aggregates all sensor data
    """
    
    def __init__(self):
        super().__init__('sense_node')
        self.get_logger().info('=== Sense Node Starting ===')
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Initialize detectors
        self.color_detector = ColorDetector()
        self.get_logger().info('Color detector initialized')
        
        self.human_detector = HumanDetector()
        if self.human_detector.model is not None:
            self.get_logger().info('Human detector (YOLO) initialized')
        else:
            self.get_logger().warn('Human detector not available - YOLO model not loaded')
        
        # State variables
        self.ultrasonic_data = {
            'front': float('inf'),
            'front_left': float('inf'),
            'front_right': float('inf'),
        }
        self.battery_level = 100.0
        self.odometry = {
            'x': 0.0,
            'y': 0.0,
            'theta': 0.0,
            'vx': 0.0,
            'vy': 0.0,
            'vtheta': 0.0
        }
        self.color_detections = {
            'colors_detected': [],
            'main_color': None,
            'targets': []
        }
        self.human_detections = {
            'person_detected': False,
            'persons': []
        }
        
        # === SUBSCRIBERS ===
        
        # Ultrasonic sensors
        self.create_subscription(
            LaserScan,
            '/ultrasonic_front/scan',
            lambda msg: self._ultrasonic_callback(msg, 'front'),
            10
        )
        self.create_subscription(
            LaserScan,
            '/ultrasonic_front_left/scan',
            lambda msg: self._ultrasonic_callback(msg, 'front_left'),
            10
        )
        self.create_subscription(
            LaserScan,
            '/ultrasonic_front_right/scan',
            lambda msg: self._ultrasonic_callback(msg, 'front_right'),
            10
        )
        
        # Camera
        self.create_subscription(
            Image,
            '/camera_front/image',
            self._camera_callback,
            10
        )
        
        # Odometry
        self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10
        )
        
        # Battery (optional - might not exist in simulation)
        self.create_subscription(
            BatteryState,
            '/battery_state',
            self._battery_callback,
            10
        )
        
        # === PUBLISHERS ===
        self.world_state_pub = self.create_publisher(String, '/sense/world_state', 10)
        
        # Debug publisher for annotated camera image
        self.debug_image_pub = self.create_publisher(Image, '/sense/debug_image', 10)
        
        # Timer to publish aggregated state
        self.create_timer(0.1, self._publish_world_state)  # 10 Hz
        
        self.get_logger().info('Sense Node ready - publishing to /sense/world_state')
    
    def _ultrasonic_callback(self, msg: LaserScan, sensor_name: str):
        """Process ultrasonic sensor data"""
        # Get minimum distance from the scan (simulates ultrasonic behavior)
        if msg.ranges:
            valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
            if valid_ranges:
                self.ultrasonic_data[sensor_name] = min(valid_ranges)
            else:
                self.ultrasonic_data[sensor_name] = float('inf')
    
    def _camera_callback(self, msg: Image):
        """Process camera image for color and human detection"""
        try:
            # Convert ROS Image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 1. Color Detection (colored targets)
            self.color_detections = self.color_detector.detect(frame)
            
            # 2. Human Detection (YOLO)
            self.human_detections = self.human_detector.detect(frame)
            
            # Publish debug image if someone is subscribed
            if self.debug_image_pub.get_subscription_count() > 0:
                debug_frame = self._draw_debug(frame)
                debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
                self.debug_image_pub.publish(debug_msg)
                
        except Exception as e:
            self.get_logger().error(f'Camera processing error: {e}')
    
    def _draw_debug(self, frame):
        """Draw debug information on frame"""
        import cv2
        
        # Draw color detections
        debug = self.color_detector.draw_detections(frame, self.color_detections)
        
        # Draw human detections
        for person in self.human_detections.get('persons', []):
            x1, y1, x2, y2 = [int(v) for v in person['bbox']]
            cv2.rectangle(debug, (x1, y1), (x2, y2), (255, 0, 255), 2)  # Magenta for persons
            cv2.putText(debug, f"Person {person['confidence']:.2f}", (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        
        # Draw ultrasonic info
        us_text = f"US: F={self.ultrasonic_data['front']:.2f} FL={self.ultrasonic_data['front_left']:.2f} FR={self.ultrasonic_data['front_right']:.2f}"
        cv2.putText(debug, us_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Draw position
        pos_text = f"Pos: x={self.odometry['x']:.2f} y={self.odometry['y']:.2f} θ={self.odometry['theta']:.2f}"
        cv2.putText(debug, pos_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return debug
    
    def _odom_callback(self, msg: Odometry):
        """Process odometry data"""
        # Extract position
        self.odometry['x'] = msg.pose.pose.position.x
        self.odometry['y'] = msg.pose.pose.position.y
        
        # Extract orientation (quaternion to yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.odometry['theta'] = np.arctan2(siny_cosp, cosy_cosp)
        
        # Extract velocities
        self.odometry['vx'] = msg.twist.twist.linear.x
        self.odometry['vy'] = msg.twist.twist.linear.y
        self.odometry['vtheta'] = msg.twist.twist.angular.z
    
    def _battery_callback(self, msg: BatteryState):
        """Process battery state"""
        self.battery_level = msg.percentage * 100  # Convert to percentage
    
    def _publish_world_state(self):
        """Publish aggregated world state to /sense/world_state"""
        
        # Build obstacles list from ultrasonic data
        obstacles = []
        for sensor, distance in self.ultrasonic_data.items():
            if distance < float('inf'):
                obstacles.append({
                    'sensor': sensor,
                    'distance': distance,
                    'type': 'ultrasonic'
                })
        
        # Check if there's an obstacle nearby (< 0.5m)
        obstacle_nearby = any(d < 0.5 for d in self.ultrasonic_data.values() if d < float('inf'))
        
        # Build world state JSON
        world_state = {
            # Battery
            'battery': self.battery_level,
            
            # Obstacles from ultrasonic sensors
            'obstacles': obstacles,
            
            # Detections summary (for Plan node compatibility)
            'detections': {
                'person': self.human_detections.get('person_detected', False),
                'valve': False,  # TODO: add valve detection if needed
                'fire': False,   # TODO: add fire detection if needed
                'obstacle': obstacle_nearby
            },
            
            # Color detection results
            'detected_color': self.color_detections.get('main_color'),
            'colors': self.color_detections.get('targets', []),
            'room_color': self.color_detections.get('main_color'),
            
            # Human detection results
            'persons': self.human_detections.get('persons', []),
            
            # Odometry / Robot position
            'odometry': self.odometry,
            'robot_state': {
                'x': self.odometry['x'],
                'y': self.odometry['y'],
                'theta': self.odometry['theta']
            }
        }
        
        # Publish as JSON string
        msg = String()
        msg.data = json.dumps(world_state)
        self.world_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SenseNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
