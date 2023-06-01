import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np


class CamPublisher(Node):
    def __init__(self):
        super().__init__('uv_camera')
        self.publisher = self.create_publisher(CompressedImage, 
            'images', 
            qos_profile=qos_profile_sensor_data
        )
        timer_period = 1/30
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.camera = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        self.width = 1080
        self.height = 720
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

    def timer_callback(self):
        success, frame = self.camera.read()
        if success:
            frame = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)
            self.get_logger().info('Publishing frame from regular camera')
            self.publisher.publish(self.bridge.cv2_to_compressed_imgmsg(frame))
        else:
            self.get_logger().info('Unsuccessful frame capture')


def main(args=None):
    rclpy.init(args=args)
    image_publisher = CamPublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
