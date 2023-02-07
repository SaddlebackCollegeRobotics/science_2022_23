import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np


class Resolution:
    width = 1280
    height = 720


class UVCameraPub(Node):
    def __init__(self):
        super().__init__('uv_cam_pub')
        self.publisher = self.create_publisher(CompressedImage, 'science_cam',
                                               qos_profile=qos_profile_sensor_data)
        timer_period = 1/30
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.camera = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        image_size = Resolution()
        image_size.width = 1280
        image_size.height = 720
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, image_size.width * 2)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, image_size.height)



    def timer_callback(self):                                                       # Check if there are subscribers
        success, frame = self.camera.read()
        if success:
            frame = np.split(frame, 2, axis=1)[0]
            frame = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)
            self.get_logger().info('Publishing science frame')
            self.publisher.publish(self.bridge.cv2_to_compressed_imgmsg(frame))
        else:
            self.get_logger().info(f'Unsuccessful frame capture')



def main(args=None):
    rclpy.init(args=args)
    cam = UVCameraPub()
    rclpy.spin(cam)
    cam.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()