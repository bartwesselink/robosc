import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Empty
from geometry_msgs.msg import Twist
import time

class SimpleMovement(Node):
    def __init__(self):
        super().__init__('simple_movement')

        self.speed_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.simple_movement_subscription = self.create_subscription(Twist, '/simple_movement', self.movement_callback, 10)

    def movement_callback(self, msg):
        self.speed_publisher.publish(msg)

        time.sleep(0.095)

        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.x = 0.0

        self.speed_publisher.publish(stop)

def main(args=None):
    rclpy.init(args=args)

    simple_movement = SimpleMovement()

    rclpy.spin(simple_movement)

    simple_movement.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()