import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageDetector(Node):
    def __init__(self):
        super().__init__('image_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info("ImageDetector node started")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Example: detect simple features or templates
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # Placeholder: just show the image
        cv2.imshow("Camera View", gray)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageDetector()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#in the future, can replace placeholder with template matching etc