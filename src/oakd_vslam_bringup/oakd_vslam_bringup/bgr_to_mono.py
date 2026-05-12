import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class BgrToMono(Node):
    def __init__(self):
        super().__init__('bgr_to_mono')
        self.declare_parameter('input_topic', '/oak/left/image_raw')
        self.declare_parameter('output_topic', '/oak/left/image_mono')
        inp = self.get_parameter('input_topic').value
        out = self.get_parameter('output_topic').value
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, out, 10)
        self.sub = self.create_subscription(Image, inp, self.cb, 10)

    def cb(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        mono = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        out = self.bridge.cv2_to_imgmsg(mono, encoding='mono8')
        out.header = msg.header
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(BgrToMono())
    rclpy.shutdown()
