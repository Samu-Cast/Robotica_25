#!/usr/bin/env python3
"""
Sense Node - Aggregates sensor data and publishes specialized ROS2 topics

Subscribes to:
    - /ultrasonic_front/scan, /ultrasonic_front_left/scan, /ultrasonic_front_right/scan (LaserScan)
    - /camera_front/image (Image) for color detection and human detection
    - /odom (Odometry) for robot position

Publishes:
    - /sense/proximity/front, front_left, front_right (Range) - ultrasonic distances
    - /sense/odometry (Pose2D) - robot position
    - /sense/detection (String) - JSON with current detection (event-driven)
    - /sense/debug_image (Image) - annotated camera view

Uses modules:
    - color_detector.py for colored target detection
    - human_detector.py for person detection (YOLO)
"""

import json
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from cv_bridge import CvBridge

# Import detection modules
try:
    from .color_detector import ColorDetector
    from .human_detector import HumanDetector
except ImportError:
    from color_detector import ColorDetector
    from human_detector import HumanDetector


class SenseNode(Node):
    """
    Main Sense Node - Publishes specialized ROS2 topics for Plan node
    """
    
    # Safety thresholds (meters)
    OBSTACLE_THRESHOLD_FRONT = 0.30
    OBSTACLE_THRESHOLD_LATERAL = 0.20
    
    # Minimum bbox area to consider a detection (filters out far detections)
    # ~5000 px² is roughly a detection at 3m distance
    MIN_BBOX_AREA = 3000
    
    # Distance estimation calibration
    # A target filling ~1/4 of the image (80000 px² on 640x480) is roughly 0.5m
    BBOX_DISTANCE_SCALE = 40000.0  # pixels² at 1 meter
    
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
        self.odometry = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.color_detections = {'targets': []}
        self.human_detections = {'person_detected': False, 'persons': []}
        
        # Last detection state (for event-driven publishing)
        self.last_detection = None
        
        # Frame skipping for CPU optimization
        self.frame_count = 0
        self.yolo_skip_frames = 5
        self.color_skip_frames = 2
        
        # Image dimensions for centering check
        self.image_width = 640
        self.image_height = 480
        
        # === SUBSCRIBERS ===
        self.create_subscription(
            LaserScan, '/ultrasonic_front/scan',
            lambda msg: self._ultrasonic_callback(msg, 'front'), 10
        )
        self.create_subscription(
            LaserScan, '/ultrasonic_front_left/scan',
            lambda msg: self._ultrasonic_callback(msg, 'front_left'), 10
        )
        self.create_subscription(
            LaserScan, '/ultrasonic_front_right/scan',
            lambda msg: self._ultrasonic_callback(msg, 'front_right'), 10
        )
        self.create_subscription(Image, '/camera_front/image', self._camera_callback, 10)
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        
        # === PUBLISHERS ===
        # Proximity sensors (Range messages)
        self.proximity_pubs = {
            'front': self.create_publisher(Range, '/sense/proximity/front', 10),
            'front_left': self.create_publisher(Range, '/sense/proximity/front_left', 10),
            'front_right': self.create_publisher(Range, '/sense/proximity/front_right', 10),
        }
        
        # Odometry (Pose2D)
        self.odometry_pub = self.create_publisher(Pose2D, '/sense/odometry', 10)
        
        # Detection (String JSON - event-driven)
        self.detection_pub = self.create_publisher(String, '/sense/detection', 10)
        
        # Debug image
        self.debug_image_pub = self.create_publisher(Image, '/sense/debug_image', 10)
        
        # Timer to publish proximity and odometry at fixed rate
        self.create_timer(0.1, self._publish_periodic)  # 10 Hz
        
        self.get_logger().info('Sense Node ready')
        self.get_logger().info('  - /sense/proximity/[front|front_left|front_right]')
        self.get_logger().info('  - /sense/odometry')
        self.get_logger().info('  - /sense/detection (event-driven)')
    
    def _ultrasonic_callback(self, msg: LaserScan, sensor_name: str):
        """Process ultrasonic sensor data"""
        if msg.ranges:
            valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
            if valid_ranges:
                self.ultrasonic_data[sensor_name] = min(valid_ranges)
            else:
                self.ultrasonic_data[sensor_name] = float('inf')
    
    def _odom_callback(self, msg: Odometry):
        """Process odometry data"""
        self.odometry['x'] = msg.pose.pose.position.x
        self.odometry['y'] = msg.pose.pose.position.y
        
        # Quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.odometry['theta'] = np.arctan2(siny_cosp, cosy_cosp)
    
    def _camera_callback(self, msg: Image):
        """Process camera image for color and human detection"""
        try:
            self.frame_count += 1
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Update image dimensions
            self.image_height, self.image_width = frame.shape[:2]
            
            # Color Detection
            if self.frame_count % self.color_skip_frames == 0:
                self.color_detections = self.color_detector.detect(frame)
            
            # Human Detection (YOLO)
            if self.frame_count % self.yolo_skip_frames == 0:
                self.human_detections = self.human_detector.detect(frame)
            
            # Process detections and publish if changed
            self._process_and_publish_detection()
            
            # Debug image
            if self.debug_image_pub.get_subscription_count() > 0:
                debug_frame = self._draw_debug(frame)
                self.debug_image_pub.publish(
                    self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
                )
                
        except Exception as e:
            self.get_logger().error(f'Camera processing error: {e}')
    
    def _calculate_bbox_area(self, bbox, format='xywh'):
        """Calculate area of bounding box
        
        Args:
            bbox: bounding box coordinates
            format: 'xywh' for [x, y, width, height] or 'xyxy' for [x1, y1, x2, y2]
        """
        if format == 'xywh':
            # [x, y, width, height] - used by color_detector
            x, y, w, h = bbox
            return abs(w * h)
        else:
            # [x1, y1, x2, y2] - used by YOLO
            x1, y1, x2, y2 = bbox
            return abs(x2 - x1) * abs(y2 - y1)
    
    def _get_bbox_center(self, bbox, format='xywh'):
        """Get center of bounding box"""
        if format == 'xywh':
            x, y, w, h = bbox
            return (x + w / 2, y + h / 2)
        else:
            x1, y1, x2, y2 = bbox
            return ((x1 + x2) / 2, (y1 + y2) / 2)
    
    def _is_centered(self, bbox, format='xywh'):
        """Check if bbox center is in the middle third of the image"""
        cx, cy = self._get_bbox_center(bbox, format)
        left_third = self.image_width / 3
        right_third = 2 * self.image_width / 3
        return left_third <= cx <= right_third
    
    def _get_horizontal_zone(self, bbox, format='xywh'):
        """Determine which horizontal zone the detection is in
        
        Returns:
            'left', 'center', or 'right'
        """
        cx, cy = self._get_bbox_center(bbox, format)
        left_third = self.image_width / 3
        right_third = 2 * self.image_width / 3
        
        if cx < left_third:
            return 'left'
        elif cx > right_third:
            return 'right'
        else:
            return 'center'
    
    def _get_proximity_for_zone(self, zone):
        """Get the ultrasonic distance for a given zone
        
        Args:
            zone: 'left', 'center', or 'right'
            
        Returns:
            distance in meters from corresponding ultrasonic sensor
        """
        if zone == 'left':
            return self.ultrasonic_data.get('front_left', float('inf'))
        elif zone == 'right':
            return self.ultrasonic_data.get('front_right', float('inf'))
        else:  # center
            return self.ultrasonic_data.get('front', float('inf'))
    
    def _estimate_distance_from_bbox(self, bbox_area):
        """Estimate distance based on bounding box area (larger = closer)
        
        Formula: distance ≈ sqrt(calibration / area)
        This gives a more realistic inverse-square relationship
        """
        if bbox_area <= 0:
            return float('inf')
        import math
        return math.sqrt(self.BBOX_DISTANCE_SCALE / bbox_area)
    
    def _process_and_publish_detection(self):
        """Determine current detection and publish if changed"""
        
        candidates = []
        
        # Check for persons (YOLO uses xyxy format)
        for person in self.human_detections.get('persons', []):
            bbox = person.get('bbox', [0, 0, 0, 0])
            area = self._calculate_bbox_area(bbox, format='xyxy')
            
            # Filter out small detections (too far)
            if area < self.MIN_BBOX_AREA:
                continue
            
            zone = self._get_horizontal_zone(bbox, format='xyxy')
            proximity_distance = self._get_proximity_for_zone(zone)
            
            candidates.append({
                'type': 'person',
                'color': None,
                'bbox_area': area,
                'zone': zone,
                'estimated_distance': self._estimate_distance_from_bbox(area),
                'proximity_distance': proximity_distance,
                'confidence': person.get('confidence', 0.0)
            })
        
        # Check for color targets (color_detector uses xywh format)
        for target in self.color_detections.get('targets', []):
            bbox = target.get('bbox', [0, 0, 0, 0])
            area = self._calculate_bbox_area(bbox, format='xywh')
            
            # Filter out small detections (too far)
            if area < self.MIN_BBOX_AREA:
                continue
            
            zone = self._get_horizontal_zone(bbox, format='xywh')
            proximity_distance = self._get_proximity_for_zone(zone)
            
            candidates.append({
                'type': 'target',
                'color': target.get('color', 'unknown'),
                'bbox_area': area,
                'zone': zone,
                'estimated_distance': self._estimate_distance_from_bbox(area),
                'proximity_distance': proximity_distance,
                'confidence': 1.0
            })
        
        # Check for obstacle (proximity-based, no visual detection)
        front_dist = self.ultrasonic_data.get('front', float('inf'))
        if front_dist < self.OBSTACLE_THRESHOLD_FRONT:
            # Only add obstacle if no visual detection is close
            if not any(c['proximity_distance'] < 0.5 for c in candidates):
                candidates.append({
                    'type': 'obstacle',
                    'color': None,
                    'bbox_area': 0,
                    'zone': 'center',
                    'estimated_distance': front_dist,
                    'proximity_distance': front_dist,
                    'confidence': 1.0
                })
        
        # Select the closest detection (smallest estimated_distance or largest bbox_area)
        if candidates:
            # Sort by bbox_area descending (larger = closer)
            candidates.sort(key=lambda x: x['bbox_area'], reverse=True)
            current_detection = candidates[0]
        else:
            current_detection = {
                'type': 'none',
                'color': None,
                'bbox_area': 0,
                'zone': None,
                'estimated_distance': float('inf'),
                'proximity_distance': float('inf'),
                'confidence': 0.0
            }
        
        # Compare with last detection - publish only if changed
        detection_key = (
            current_detection['type'],
            current_detection['color'],
            current_detection['zone']
        )
        
        if self.last_detection != detection_key:
            self.last_detection = detection_key
            
            # Publish detection
            msg = String()
            # Handle inf for JSON serialization
            det_copy = current_detection.copy()
            if det_copy['estimated_distance'] == float('inf'):
                det_copy['estimated_distance'] = -1.0
            msg.data = json.dumps(det_copy)
            self.detection_pub.publish(msg)
            
            self.get_logger().info(
                f"Detection: {current_detection['type']} "
                f"(color={current_detection['color']}, zone={current_detection['zone']})"
            )
    
    def _publish_periodic(self):
        """Publish proximity and odometry at fixed rate"""
        
        # Publish proximity sensors
        for sensor_name, distance in self.ultrasonic_data.items():
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f'ultrasonic_{sensor_name}'
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.26  # ~15 degrees
            msg.min_range = 0.02
            msg.max_range = 4.0
            msg.range = distance if distance < float('inf') else -1.0
            self.proximity_pubs[sensor_name].publish(msg)
        
        # Publish odometry
        odom_msg = Pose2D()
        odom_msg.x = self.odometry['x']
        odom_msg.y = self.odometry['y']
        odom_msg.theta = self.odometry['theta']
        self.odometry_pub.publish(odom_msg)
    
    def _draw_debug(self, frame):
        """Draw debug information on frame"""
        import cv2
        
        debug = self.color_detector.draw_detections(frame, self.color_detections)
        
        # Draw human detections
        for person in self.human_detections.get('persons', []):
            x1, y1, x2, y2 = [int(v) for v in person['bbox']]
            cv2.rectangle(debug, (x1, y1), (x2, y2), (255, 0, 255), 2)
            cv2.putText(debug, f"Person {person['confidence']:.2f}", (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        
        # Draw sensor info
        us_text = f"US: F={self.ultrasonic_data['front']:.2f} FL={self.ultrasonic_data['front_left']:.2f} FR={self.ultrasonic_data['front_right']:.2f}"
        cv2.putText(debug, us_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        pos_text = f"Pos: x={self.odometry['x']:.2f} y={self.odometry['y']:.2f} θ={self.odometry['theta']:.2f}"
        cv2.putText(debug, pos_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Draw current detection
        det_text = f"Detection: {self.last_detection}"
        cv2.putText(debug, det_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        return debug


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
