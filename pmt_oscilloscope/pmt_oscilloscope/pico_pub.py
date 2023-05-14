# ========================================================================================
# Author:   Jasper Doan     @wluxie
# Date:     05/14/2023
# ========================================================================================


import rclpy                                # ROS2 Python API
from rclpy.node import Node                 # ROS2 Node API
from std_msgs.msg import Float64MultiArray  # ROS2 Float64MultiArray message type
from .lib.picoscope import Oscilloscope     # Oscilloscope wrapper class


class PicoPub(Node):
    TIMER_PERIOD = 10.0     # Timer period in seconds

    # '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    # Constructor
    # ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,, 
    def __init__(self):
        super().__init__('pico_pub')    # Create node with name 'controller_pub'

        # Publishing                                [type]                [topic]     [queue_size]
        self.publisher_ = self.create_publisher(Float64MultiArray, 'science/pico_data', 10) # Create publisher to publish controller input
        self.timer = self.create_timer(PicoPub.TIMER_PERIOD, self.timer_callback)           # Create timer to publish controller input

        # Oscilloscope
        self.osc = Oscilloscope()   # Create oscilloscope object


    # '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    # Timer Callback
    #       This function is called every time the timer goes off. It gets the controller
    #       input and publishes it to the ROS2 topic 'controls'
    # ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
    def timer_callback(self):
        msg = Float64MultiArray()                   # Create message

        print('Capturing data...')
        self.osc.open_unit()                        # Open unit
        self.osc.capture()                          # Capture data
        time, adc2mVChA, _ = self.osc.get_data()    # Get data
        self.osc.close_unit()                       # Close unit

        print('Publishing data...\n\n')
        for t, v in zip(time, adc2mVChA):           # Append data to message
            msg.data.append(t)                      #   Time
            msg.data.append(v)                      #   Channel A Voltages
        self.publisher_.publish(msg)                # Publish message



def main(args=None):
    rclpy.init(args=args)

    pico_pub = PicoPub()

    rclpy.spin(pico_pub)

    pico_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()