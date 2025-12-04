"""
    Perception Node for Charlie Robot
    Handles sensor data processing and publishes perception results
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class PerceptionNode(Node):
    """
    ROS2 Node for perception processing
    Subscribes to camera feeds and publishes detection results
    """

    def __init__(self):
        super().__init__('perception_node')
        
        #Publishers for perception results
        self.fire_pub = self.create_publisher(Bool, 'perception/fire_detected', 10)
        self.valve_pub = self.create_publisher(Bool, 'perception/valve_visible', 10)
        self.person_pub = self.create_publisher(Bool, 'perception/person_visible', 10)
        self.temp_pub = self.create_publisher(Float32, 'perception/temperature', 10)
        
        #Subscribers to sensor data
        self.rgb_sub = self.create_subscription(
            Image,
            'camera/rgb/image_raw',
            self.rgb_callback,
            10
        )
        
        #CV Bridge for image conversion
        self.bridge = CvBridge()
        
        #Timer for periodic processing
        self.timer = self.create_timer(0.1, self.perception_loop)
        
        self.get_logger().info('Perception Node initialized')

    def rgb_callback(self, msg):
        """Process RGB camera images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            #Process image here (fire detection, valve detection, etc.)
            #This is a placeholder for actual computer vision algorithms
            
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def perception_loop(self):
        """Main perception processing loop"""
        #Placeholder: In real implementation, this would process
        #sensor data and publish detection results
        
        #Example: Publish dummy data
        fire_msg = Bool()
        fire_msg.data = False
        self.fire_pub.publish(fire_msg)
        
        valve_msg = Bool()
        valve_msg.data = False
        self.valve_pub.publish(valve_msg)
        
        person_msg = Bool()
        person_msg.data = False
        self.person_pub.publish(person_msg)
        
        temp_msg = Float32()
        temp_msg.data = 25.0  #Normal temperature
        self.temp_pub.publish(temp_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
