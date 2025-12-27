#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import sys
import select
import termios
import tty

class EmergencyControlNode(Node):
    def __init__(self):
        super().__init__('emergency_control_node')

        # Publishers
        self.pub_stop = self.create_publisher(Twist, '/cmd_vel_stop', 10)
        self.pub_resume = self.create_publisher(Twist, '/cmd_vel_coverage', 10)

        self.get_logger().info('Emergency Control Node Started.')
        self.get_logger().info('Press SPACE to stop, ENTER to resume coverage, Q to quit.')

        # Start keyboard thread
        self.thread = threading.Thread(target=self.keyboard_loop)
        self.thread.daemon = True
        self.thread.start()

    def keyboard_loop(self):
        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        try:
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == ' ':
                        self.send_stop()
                    elif key == '\r' or key == '\n':
                        self.send_resume()
                    elif key.lower() == 'q':
                        self.get_logger().info('Quitting emergency control node.')
                        rclpy.shutdown()
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def send_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        for _ in range(5):  # publish a few times to guarantee stop
            self.pub_stop.publish(twist)
        self.get_logger().info('STOP command sent.')

    def send_resume(self):
        twist = Twist()
        twist.linear.x = 0.1  # minimal forward speed to resume coverage
        twist.angular.z = 0.0
        self.pub_resume.publish(twist)
        self.get_logger().info('RESUME command sent.')

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
