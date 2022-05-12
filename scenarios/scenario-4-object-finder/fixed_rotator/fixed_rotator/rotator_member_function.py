import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import numpy
import time
from tf_transformations import euler_from_quaternion

class FixedRotator(Node):
    current_rotation = None
    goal = None
    left = None
    
    def __init__(self):
        super().__init__('fixed_rotator')

        self.rotate = self.create_publisher(Twist, '/cmd_vel', 1)
        self.done = self.create_publisher(Empty, '/rotate_done', 1)
        self.odom = self.create_subscription(Odometry, '/odom', self.callback_odom, 1)
        self.left = self.create_subscription(Empty, '/rotate_left', self.callback_left, 1)
        self.right = self.create_subscription(Empty, '/rotate_right', self.callback_right, 1)

    def callback_left(self, req):
        self.left = True
        msg = Twist()
        msg.angular.z = 0.02
        self.rotate.publish(msg)

        start_rotation = self.current_rotation
        self.goal = self.normalize(start_rotation + math.pi * 0.5)

    def callback_right(self, req):
        self.left = False
        msg = Twist()
        msg.angular.z = -0.02
        self.rotate.publish(msg)

        start_rotation = self.current_rotation
        self.goal = self.normalize(start_rotation - math.pi * 0.5)

    def normalize(self, value):
        return value % (2 * math.pi) - math.pi # normalize to [-pi, pi]

    def callback_odom(self, msg):
        orientation = msg.pose.pose.orientation

        x, y, z = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        self.current_rotation = z

        if self.goal != None:
            if (self.normalize(self.goal - self.current_rotation) < 0 and self.left == True) or (self.normalize(self.current_rotation - self.goal) < 0 and self.left == False):
                msg = Twist()
                msg.angular.z = 0.0
                self.rotate.publish(msg)
                
                msg = Empty()
                self.done.publish(msg)
                
                self.goal = None


def main(args=None):
    rclpy.init(args=args)

    fixed_rotator = FixedRotator()

    rclpy.spin(fixed_rotator)

    fixed_rotator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()