# source: http://edu.gaitech.hk/turtlebot/line-follower.html

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Empty
from sensor_msgs.msg import Image

import cv2, cv_bridge
import numpy

class LineDetector(Node):
    lower_yellow = numpy.array([10, 10, 10])
    upper_yellow = numpy.array([255, 255, 250])
    
    def __init__(self):
        super().__init__('line_detector')

        self.bridge = cv_bridge.CvBridge()

        self.correction_publisher = self.create_publisher(Float32, '/correction', 10)
        self.no_line_publisher = self.create_publisher(Empty, '/no_line', 10)
        self.camera_subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert color type
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)

        height, width, depth = image.shape

        # Define the area that we are considering in our search
        area_top = round(3 * height / 4)
        area_bottom = round(3 * height / 4 + 20)

        mask[0:area_top, 0:width] = 0
        mask[area_bottom:height, 0:width] = 0

        # Check if we can find a line and determine the center point
        M = cv2.moments(mask)
        if M['m00'] > 0:
                line_center_x = M['m10'] / M['m00']
                current_error = line_center_x - float(width) / 2.0

                msg = Float32()
                msg.data = current_error
                self.correction_publisher.publish(msg)
        else:
            msg = Empty()
            self.no_line_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    line_detector = LineDetector()

    rclpy.spin(line_detector)

    line_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()