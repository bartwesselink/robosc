# source: http://edu.gaitech.hk/turtlebot/line-follower.html

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
import tkinter

class EmergencyStop(Node):
    def __init__(self):
        super().__init__('line_detector')

        self.stop_publisher = self.create_publisher(Empty, '/stop', 10)
        self.continue_publisher = self.create_publisher(Empty, '/continue', 10)

        self.show_window()

    def stop_callback(self):
        self.stop_button["state"] = "disabled"
        self.continue_button["state"] = "normal"

        msg = Empty()
        self.stop_publisher.publish(msg)

    def continue_callback(self):
        self.stop_button["state"] = "normal"
        self.continue_button["state"] = "disabled"

        msg = Empty()
        self.continue_publisher.publish(msg)

    def show_window(self):
        top = tkinter.Tk()
        top.title("Emergency Stop")

        self.stop_button = tkinter.Button(top, text = "Stop", command = self.stop_callback, width = 100, height = 20)
        self.continue_button = tkinter.Button(top, text = "Continue", command = self.continue_callback, width = 100, height = 20)

        self.continue_button["state"] = "disabled"

        self.stop_button.pack()
        self.continue_button.pack()

        top.mainloop()



def main(args=None):
    rclpy.init(args=args)

    emergency_stop = EmergencyStop()

    rclpy.spin(emergency_stop)

    emergency_stop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()