# partially based on: http://edu.gaitech.hk/turtlebot/line-follower.html

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Empty
from sensor_msgs.msg import Image

import cv2, cv_bridge
import numpy

class GoalDetector(Node):
    def __init__(self):
        super().__init__('goal_detector')

        self.bridge = cv_bridge.CvBridge()

        self.correction_publisher = self.create_publisher(Float32, '/goal_correction', 10)
        self.no_goal_publisher = self.create_publisher(Empty, '/no_goal', 10)
        self.camera_subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert color type
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, numpy.array([52, 0, 55]), numpy.array([104, 255, 255]))

        height, width, depth = image.shape

        # Check if we can find a goal and determine the center point
        M = cv2.moments(mask)
        if M['m00'] > 0:
                goal_center_x = M['m10'] / M['m00']
                current_error = goal_center_x - float(width) / 2.0

                msg = Float32()
                msg.data = current_error
                self.correction_publisher.publish(msg)
        else:
            msg = Empty()
            self.no_goal_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    goal_detector = GoalDetector()

    rclpy.spin(goal_detector)

    goal_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()