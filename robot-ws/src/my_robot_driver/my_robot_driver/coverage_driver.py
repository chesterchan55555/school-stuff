import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ZigZagCoverage:
    """
    Simple zig-zag coverage algorithm.
    Moves forward for N steps, then turns left or right alternately.
    """
    def __init__(self, forward_steps=20, turn_steps=5):
        self.forward_steps = forward_steps
        self.turn_steps = turn_steps
        self.step = 0
        self.turning = False
        self.turn_left = True  # alternate turns

    def get_next_twist(self) -> Twist:
        twist = Twist()

        if not self.turning:
            # Move forward
            twist.linear.x = 0.1
            twist.angular.z = 0.0
            self.step += 1
            if self.step >= self.forward_steps:
                self.turning = True
                self.step = 0
        else:
            # Turn in place
            twist.linear.x = 0.0
            twist.angular.z = 0.5 if self.turn_left else -0.5
            self.step += 1
            if self.step >= self.turn_steps:
                self.turning = False
                self.step = 0
                self.turn_left = not self.turn_left  # alternate for next row

        return twist

class CoverageDriver(Node):
    def __init__(self):
        super().__init__('coverage_driver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/stop_cmd', self.stop_callback, 10)
        self.create_subscription(Twist, '/start_cmd', self.start_callback, 10)

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.publish_twist)

        self.running = True
        self.algorithm = ZigZagCoverage()  # Use the zig-zag pattern

        self.get_logger().info("CoverageDriver initialized with zig-zag pattern.")

    def stop_callback(self, msg):
        self.running = False
        self.get_logger().info("Received STOP command.")

    def start_callback(self, msg):
        self.running = True
        self.get_logger().info("Received START command.")

    def publish_twist(self):
        twist = Twist()
        if self.running:
            twist = self.algorithm.get_next_twist()
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CoverageDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
