#!/usr/bin/env python3
"""
Color Detector module for identifying colored rectangular targets.
Supports standalone ROS 2 execution or integration as a module.
"""

import cv2
import numpy as np


class ColorDetector:
    """
        Detects and localizes colored rectangular targets in BGR images.
    """
    
    def __init__(self):
        #HSV ranges for target identification
        self.target_colors = {
            'green': {
                'lower': np.array([40, 30, 30]),
                'upper': np.array([80, 255, 255]),
                'draw_color': (0, 255, 0)
            },
            'blue': {
                'lower': np.array([100, 150, 0]),
                'upper': np.array([140, 255, 255]),
                'draw_color': (255, 0, 0)
            },
            'red': {
                'lower': np.array([0, 150, 100]),
                'upper': np.array([10, 255, 255]),
                'lower2': np.array([160, 150, 100]),
                'upper2': np.array([180, 255, 255]),
                'draw_color': (0, 0, 255)
            }
        }
        self.min_area = 2000
    
    def detect(self, frame):
        """
        Detect colored targets in a frame.
        
        Args:
            frame: OpenCV BGR image.
            
        Returns:
            Dict containing detected colors, the dominant color, and target metadata.
        """
        if frame is None:
            return self._empty_result()
        
        try:
            #Pre-processing
            blurred = cv2.GaussianBlur(frame, (5, 5), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            
            targets = []
            colors_detected = set()
            
            for color_name, params in self.target_colors.items():
                #Create mask for this color
                mask = cv2.inRange(hsv, params['lower'], params['upper'])
                
                #If second range exists, combine masks
                if 'lower2' in params:
                    mask2 = cv2.inRange(hsv, params['lower2'], params['upper2'])
                    mask = cv2.bitwise_or(mask, mask2)
                
                #Clean up mask
                kernel = np.ones((5, 5), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                
                #Find contours
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    
                    if area > self.min_area:
                        #Approximate polygon
                        perimeter = cv2.arcLength(cnt, True)
                        approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)
                        
                        #Check if it's a rectangle (4 vertices)
                        is_rectangle = len(approx) == 4
                        
                        #Get bounding box
                        x, y, w, h = cv2.boundingRect(cnt)
                        cx = x + w // 2
                        cy = y + h // 2
                        
                        targets.append({
                            'color': color_name,
                            'area': area,
                            'bbox': [x, y, w, h],
                            'center': (cx, cy),
                            'is_rectangle': is_rectangle
                        })
                        
                        colors_detected.add(color_name)
            
            #Sort by area (largest first)
            targets.sort(key=lambda x: x['area'], reverse=True)
            
            #Main color is the largest target
            main_color = targets[0]['color'] if targets else None
            
            return {
                'colors_detected': list(colors_detected),
                'main_color': main_color,
                'targets': targets
            }
            
        except Exception as e:
            print(f"Color detection error: {e}")
            return self._empty_result()
    
    def _empty_result(self):
        """Return empty detection result"""
        return {
            'colors_detected': [],
            'main_color': None,
            'targets': []
        }
    
    def draw_detections(self, frame, results):
        """
        Draw detection overlays on the frame.
        
        Args:
            frame: Base image.
            results: Detection metadata.
            
        Returns:
            Annotated image.
        """
        annotated = frame.copy()
        
        for target in results.get('targets', []):
            x, y, w, h = target['bbox']
            color_name = target['color']
            
            #Get draw color
            draw_color = self.target_colors.get(color_name, {}).get('draw_color', (255, 255, 255))
            
            #Draw rectangle
            cv2.rectangle(annotated, (x, y), (x + w, y + h), draw_color, 2)
            
            #Draw label
            label = f"{color_name}"
            if target['is_rectangle']:
                label += " [Rect]"
            cv2.putText(annotated, label, (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 2)
        
        return annotated


#Singleton instance for easy import
_detector_instance = None

def get_color_detector():
    """Get singleton instance of ColorDetector"""
    global _detector_instance
    if _detector_instance is None:
        _detector_instance = ColorDetector()
    return _detector_instance


#ROS2 Node (optional, for standalone use)

def main(args=None):
    """Run as standalone ROS2 node"""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    
    class ColorDetectorNode(Node):
        def __init__(self):
            super().__init__('color_detector')
            
            self.detector = ColorDetector()
            self.bridge = CvBridge()
            
            self.subscription = self.create_subscription(
                Image,
                '/camera_front/image',
                self.callback,
                10
            )
            
            self.debug_pub = self.create_publisher(Image, '/camera/color', 10)
            self.get_logger().info("Color Detector Node started")
        
        def callback(self, msg):
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                results = self.detector.detect(frame)
                
                if results['main_color']:
                    self.get_logger().info(f"Detected: {results['colors_detected']}")
                
                #Publish debug image
                annotated = self.detector.draw_detections(frame, results)
                out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                self.debug_pub.publish(out_msg)
                
            except Exception as e:
                self.get_logger().error(f'Error: {e}')
    
    rclpy.init(args=args)
    node = ColorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()