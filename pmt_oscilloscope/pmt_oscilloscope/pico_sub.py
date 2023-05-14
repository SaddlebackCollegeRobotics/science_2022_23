# ========================================================================================
# Author:   Jasper Doan     @wluxie
# Date:     05/14/2023
# ========================================================================================


import rclpy                                # ROS2 Python API
from rclpy.node import Node                 # ROS2 Node API
from std_msgs.msg import Float64MultiArray  # ROS2 Float64MultiArray message type


class PicoSub(Node):
    # '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    # Constructor
    # ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,, 
    def __init__(self):
        super().__init__('pico_sub')

        self.subscription = self.create_subscription(           # Create subscription to 'controls' topic
            Float64MultiArray,                                  # Message type
            'drive/analog_control',                             # Topic name
            self.listener_callback,                             # Callback function to call when message is received
            10)                                                 # Queue size


    # '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    # Listener Callback
    #       Called when a message is received on the 'controls' topic
    # ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
    def listener_callback(self, msg):
        raise NotImplementedError



def main(args=None):
    rclpy.init(args=args)

    pico_sub = PicoSub()

    rclpy.spin(pico_sub)

    pico_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()