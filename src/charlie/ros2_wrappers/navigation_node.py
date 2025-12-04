"""
    Navigation Node for Charlie Robot
    Path planning and obstacle avoidance

    TODO: Implement navigation algorithms
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class NavigationNode(Node):
    """ROS2 Node for navigation and path planning"""
    
    def __init__(self):
        super().__init__('navigation_node')
        
        #Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        #Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.get_logger().info('🧭 Navigation Node initialized')
    
    def odom_callback(self, msg):
        """Process odometry data"""
        #TODO: Implement navigation logic
        pass
    
    def move_forward(self, speed=0.2):
        """Move robot forward"""
        cmd = Twist()
        cmd.linear.x = speed
        self.cmd_vel_pub.publish(cmd)
    
    def rotate(self, angular_speed=0.5):
        """Rotate robot"""
        cmd = Twist()
        cmd.angular.z = angular_speed
        self.cmd_vel_pub.publish(cmd)
    
    def stop(self):
        """Stop robot"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
