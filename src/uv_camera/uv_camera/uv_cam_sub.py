import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
from datetime import datetime
import numpy as np


class UVCameraSub(Node):
    def __init__(self):
        super().__init__('uv_cam_sub')                                                                             # Create the node                                                         # Create a callback group

        self.callback_group = ReentrantCallbackGroup()
        self.subscription = self.create_subscription(CompressedImage, 'science_cam',
                                                     self.listener_callback, qos_profile=qos_profile_sensor_data, callback_group=self.callback_group)   
        
        self.bridge = CvBridge()
        frame_width = 640
        frame_height = 360
        self.frame = np.zeros([frame_height, frame_width, 4], dtype=np.uint8)

        self.gui = False
        self.verbose = False



    def listener_callback(self, data):
        self.frame = self.bridge.compressed_imgmsg_to_cv2(data)

        if self.verbose:
            self.get_logger().info('Receiving science frame')
        if self.gui:
            cv2.imshow('UV Camera View', self.frame)
            cv2.waitKey(1)




def main(args=None):
    rclpy.init(args=args)
    cam = UVCameraSub()
    cam.gui = True
    cv2.namedWindow('UV Camera View', cv2.WINDOW_KEEPRATIO)
    rclpy.spin(cam)
    cam.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()