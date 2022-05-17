# partially based on: http://edu.gaitech.hk/turtlebot/line-follower.html

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Empty
from sensor_msgs.msg import Image

import cv2, cv_bridge
import numpy

class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector')

        self.bridge = cv_bridge.CvBridge()

        self.correction_publisher = self.create_publisher(Float32, '/ball_correction', 10)
        self.needs_adjustment = self.create_publisher(Empty, '/needs_ajustment', 10)
        self.no_adjustment = self.create_publisher(Empty, '/no_adjustment', 10)
        self.no_ball_publisher = self.create_publisher(Empty, '/no_ball', 10)
        self.ball_front_check_publisher = self.create_publisher(Empty, '/ball_front_check', 10)
        self.camera_subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert color type
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask_one = cv2.inRange(hsv, numpy.array([0, 70, 50]), numpy.array([10, 255, 255]))
        mask_two = cv2.inRange(hsv, numpy.array([170, 70, 50]), numpy.array([180, 255, 255]))
        mask_three = cv2.inRange(hsv, numpy.array([0, 0, 0]), numpy.array([180,255,40]))

        mask = mask_one + mask_two + mask_three

        height, width, depth = image.shape

        # Check if we can find a ball and determine the center point
        M = cv2.moments(mask)
        
        if M['m00'] > 0:
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                contour = contours[0]
                x, y, w, h = cv2.boundingRect(contour)

                portion_size = w / float(width)

                if portion_size > 0.9:
                    msg = Empty()
                    self.ball_front_check_publisher.publish(msg)

            ball_center_x = M['m10'] / M['m00']
            current_error = ball_center_x - float(width) / 2.0

            fraction = abs((ball_center_x) / float(width))

            msg = Float32()
            msg.data = current_error
            self.correction_publisher.publish(msg)

            if fraction > 0.6 or fraction < 0.4:
                msg = Empty()
                self.needs_adjustment.publish(msg)
            else:
                msg = Empty()
                self.no_adjustment.publish(msg)

        else:
            msg = Empty()
            self.no_ball_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    ball_detector = BallDetector()

    rclpy.spin(ball_detector)

    ball_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()