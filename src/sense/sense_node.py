#!/usr/bin/env python3
"""
Sense Node - Aggregates sensor data and publishes specialized ROS2 topics

Subscribes to:
    - /ir_intensity (IrIntensityVector) - IR proximity sensors from iCreate3
    - /camera_front/image (Image) for color detection and human detection
    - /odom (Odometry) for robot position

Publishes:
    - /sense/proximity/front, front_left, front_right (Range) - IR proximity values
    - /sense/odometry (Pose2D) - robot position
    - /sense/detection (String) - JSON with current detection (event-driven)
    - /sense/detection_zone (String) - detection zone (left, center, right)
    - /sense/debug_image (Image) - annotated camera view
    - /sense/battery (Float32) - battery level

Uses modules:
    - color_detector.py for colored target detection
    - human_detector.py for person detection (YOLO)
"""

import json
import numpy as np

import os
os.environ.setdefault("TORCH_CPP_LOG_LEVEL", "ERROR")
os.environ.setdefault("KMP_WARNINGS", "0")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Range, BatteryState
from irobot_create_msgs.msg import IrIntensityVector
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String, Float32, Bool
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
    
    # Map iCreate3 IR sensor frame_ids to internal sensor names
    # ir_intensity_front_center_left  -> front (center-facing on our robot)
    # ir_intensity_left -> front_left
    # ir_intensity_right -> front_right
    IR_SENSOR_MAP = {
        'ir_intensity_front_center_left': 'front',
        'ir_intensity_left': 'front_left',
        'ir_intensity_right': 'front_right',
    }

    # IR Intensity -> Distance conversion (threshold-based, calibrated from physical robot)
    # Thresholds for IR intensity bands:
    #   0-70    -> far (no concern)   -> 0.20m (20cm)
    #   70-280  -> valid range        -> 0.12m (12cm)
    #   >280    -> danger (too close) -> 0.05m (5cm)
    IR_THRESHOLD_LOW = 70      # Below this = far away
    IR_THRESHOLD_HIGH = 280    # Above this = dangerously close
    IR_DIST_FAR = 0.20         # 20 cm - far, no concern
    IR_DIST_MEDIUM = 0.12      # 12 cm - valid detection range
    IR_DIST_CLOSE = 0.05       # 5 cm  - danger, too close
    IR_MAX_DISTANCE = 0.20     # meters - max reported distance

    # Minimum bbox area to consider a detection (filters out far detections)
    # ~5000 px² is roughly a detection at 3m distance
    MIN_BBOX_AREA = 2000
    
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
        
        # State variables - distances in meters (converted from IR intensity)
        self.proximity_data = {
            'front': float('inf'),
            'front_left': float('inf'),
            'front_right': float('inf'),
        }
        self.odometry = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.battery_level = None  # None until first message received
        self.color_detections = {'targets': []}
        self.human_detections = {'person_detected': False, 'persons': []}
        
        # Last detection state (for event-driven publishing)
        self.last_detection = None
        self.last_detection_zone = None  # Track detection zone for continuous publishing
        
        # Frame skipping for CPU optimization
        self.frame_count = 0
        self.yolo_skip_frames = 5
        self.color_skip_frames = 2
        
        # Image dimensions for centering check
        self.image_width = 640
        self.image_height = 480
        
        # === SUBSCRIBERS ===
        # QoS for iCreate3 sensor topics (BEST_EFFORT required by the physical robot)
        from rclpy.qos import qos_profile_sensor_data
        
        # IR intensity sensors from iCreate3 physical robot
        self.create_subscription(
            IrIntensityVector, '/ir_intensity',
            self._ir_intensity_callback, qos_profile_sensor_data
        )
        self.create_subscription(Image, '/camera_front/image', self._camera_callback, 10)
        self.create_subscription(Odometry, '/odom', self._odom_callback, qos_profile_sensor_data)
        self.create_subscription(BatteryState, '/battery_state', self._battery_callback, qos_profile_sensor_data)
        
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
        
        # Detection Zone (String - continuous: 'left', 'center', 'right', 'none')
        # This allows Plan node to know where detection is in camera frame for centering
        self.detection_zone_pub = self.create_publisher(String, '/sense/detection_zone', 10)
        
        # Debug image - publish with larger queue
        self.debug_image_pub = self.create_publisher(Image, '/sense/debug_image', 30)
        
        # Battery (Float32 percentage)
        self.battery_pub = self.create_publisher(Float32, '/sense/battery', 10)
        
        # Timer to publish proximity and odometry at fixed rate
        self.create_timer(0.1, self._publish_periodic)  # 10 Hz
        
        self.get_logger().info('Sense Node ready')
        self.get_logger().info('  - /sense/proximity/[front|front_left|front_right] (from IR sensors)')
        self.get_logger().info('  - /sense/odometry')
        self.get_logger().info('  - /sense/battery')
        self.get_logger().info('  - /sense/detection (event-driven)')
        self.get_logger().info('  - /sense/detection_zone (continuous: left/center/right/none)')
    
    def _ir_to_distance(self, ir_value):
        """Convert IR intensity value to distance in meters using threshold bands.
        
        Threshold-based conversion calibrated for iCreate3 IR sensors:
            IR 0-70   -> 0.20m (20cm) - far, no concern
            IR 70-280 -> 0.12m (12cm) - valid detection range
            IR >280   -> 0.05m (5cm)  - danger, too close
        
        Args:
            ir_value: raw IR intensity (int16, 0 = no object, higher = closer)
        Returns:
            distance in meters (0.20, 0.12, or 0.05)
        """
        if ir_value <= self.IR_THRESHOLD_LOW:
            return self.IR_DIST_FAR       # Far away - no concern
        elif ir_value <= self.IR_THRESHOLD_HIGH:
            return self.IR_DIST_MEDIUM    # Valid range
        else:
            return self.IR_DIST_CLOSE     # Too close - danger
    
    def _ir_intensity_callback(self, msg: IrIntensityVector):
        """Process IR intensity sensor data from iCreate3
        
        The IrIntensityVector contains 7 readings.
        Each reading has header.frame_id (sensor name) and value (int16).
        Higher value = closer object -> converted to distance in meters.
        """
        # One-time debug: log all frame_ids on first message
        if not hasattr(self, '_ir_debug_done'):
            self._ir_debug_done = True
            all_sensors = [(r.header.frame_id, r.value) for r in msg.readings]
            self.get_logger().info(f'IR SENSORS RECEIVED ({len(msg.readings)}): {all_sensors}')
        
        for reading in msg.readings:
            frame_id = reading.header.frame_id
            if frame_id in self.IR_SENSOR_MAP:
                sensor_name = self.IR_SENSOR_MAP[frame_id]
                self.proximity_data[sensor_name] = self._ir_to_distance(reading.value)
    
    def _odom_callback(self, msg: Odometry):
        """Process odometry data"""
        self.odometry['x'] = msg.pose.pose.position.x
        self.odometry['y'] = msg.pose.pose.position.y
        
        # Quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.odometry['theta'] = np.arctan2(siny_cosp, cosy_cosp)
    
    def _battery_callback(self, msg: BatteryState):
        """Process battery state"""
        # BatteryState.percentage is 0.0-1.0, convert to 0-100
        self.battery_level = msg.percentage * 100.0
    
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
        """Get the distance in meters for a given zone
        
        Args:
            zone: 'left', 'center', or 'right'
            
        Returns:
            distance in meters from corresponding IR sensor
        """
        if zone == 'left':
            return self.proximity_data.get('front_left', float('inf'))
        elif zone == 'right':
            return self.proximity_data.get('front_right', float('inf'))
        else:  # center
            return self.proximity_data.get('front', float('inf'))
    
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
        
        # Publish proximity as distances in meters (converted from IR intensity)
        for sensor_name, distance in self.proximity_data.items():
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f'ir_{sensor_name}'
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = 0.26  # ~15 degrees
            msg.min_range = self.IR_DIST_CLOSE
            msg.max_range = self.IR_MAX_DISTANCE
            msg.range = distance if distance < float('inf') else -1.0
            self.proximity_pubs[sensor_name].publish(msg)
        
        # Publish odometry
        odom_msg = Pose2D()
        odom_msg.x = self.odometry['x']
        odom_msg.y = self.odometry['y']
        odom_msg.theta = self.odometry['theta']
        self.odometry_pub.publish(odom_msg)
        
        # Publish battery (only if we have received data)
        if self.battery_level is not None:
            battery_msg = Float32()
            battery_msg.data = self.battery_level
            self.battery_pub.publish(battery_msg)
        
        # Publish detection zone continuously (for color centering in Plan node)
        # Determine zone from LATEST detections (color or human)
        current_zone = 'none'
        
        # Check color targets
        for target in self.color_detections.get('targets', []):
            bbox = target.get('bbox', [0, 0, 0, 0])
            area = self._calculate_bbox_area(bbox, format='xywh')
            if area >= self.MIN_BBOX_AREA:
                current_zone = self._get_horizontal_zone(bbox, format='xywh')
                break
        
        # Check humans (lower priority, only if no color detected)
        if current_zone == 'none':
            for person in self.human_detections.get('persons', []):
                bbox = person.get('bbox', [0, 0, 0, 0])
                area = self._calculate_bbox_area(bbox, format='xyxy')
                if area >= self.MIN_BBOX_AREA:
                    current_zone = self._get_horizontal_zone(bbox, format='xyxy')
                    break
        
        zone_msg = String()
        zone_msg.data = current_zone
        self.detection_zone_pub.publish(zone_msg)
    
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
        
        # Draw sensor info (distances in meters, converted from IR)
        ir_text = f"IR: F={self.proximity_data['front']:.2f}m FL={self.proximity_data['front_left']:.2f}m FR={self.proximity_data['front_right']:.2f}m"
        cv2.putText(debug, ir_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
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
