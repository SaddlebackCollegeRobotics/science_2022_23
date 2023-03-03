
# Ros2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray

# General imports
from time import perf_counter
import math
from .funnel_cake_controller import *
import signal


class MinimalSubscriber(Node):

    # Class init
    def __init__(self):

        # Signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signalHandler)

        # Motor controller for turret
        # self.mcp3 = Motor_Controller(
        #     rc = Roboclaw(COMPORT_NAME_1, 115200),
        #     address = 0x80  
        # )

        # Give the node a name.
        super().__init__('funnel_cake_subscriber')

        # Subscribe to the topic 'topic'. Callback gets called when a message is received.
        self.subscription = self.create_subscription(
            Int64MultiArray,
            '/science/funnel_cake',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.timer_period = 0.1 # Interval for checking heartbeat.
        self.signal_timeout = 3 # Max time before soft stop if subscription heartbeat is not detected.

        # self.time_of_last_callback = perf_counter()
        # self.timer = self.create_timer(self.timer_period, self.Subscription_HeartBeat)


    # For safety. Stop motors if subscription not being received.
    def Subscription_HeartBeat(self):
        
        # if perf_counter() - self.time_of_last_callback > self.signal_timeout:
        #     self.SoftStop()
        ...


    # Callback for Ctrl+C
    def signalHandler(self):

        print("\nExited Cleanly")
        self.SoftStop()
        quit()


    # Signal to stop all motors
    def SoftStop(self):
        
        print("Soft Stop Triggered")

        self.mcp3.rc.ForwardM1(self.mcp3.address, 0)
        self.mcp3.rc.ForwardM2(self.mcp3.address, 0)


    # called every time the subscriber receives a message
    def listener_callback(self, msg):

        data_list = []
        for i in range(6):
            data_list.append(msg.data[i])

        # self.time_of_last_callback = perf_counter()
        (turret_rotation, allow_rotation, reset_encoders, reset_position,
        print_encoders, stop_motor) = msg.data
        

        if allow_rotation == 1:
            set_turret_rotation(self.mcp3, turret_rotation)
            print("set rotation")

        elif reset_encoders == 1:
            self.mcp3.reset_encoders()
            print("reset encoder")


        elif reset_position == 1:
            set_turret_rotation(self.mcp3, 0)
            print("reset rotation")


        elif print_encoders == 1:
            print(self.mcp3.print_encoders())
            print("print encoders")


        elif stop_motor == 1:
            self.SoftStop()
            print("stop motors")


        del data_list

        


        # This prints an info message to the console, along with the data it received. 
        # for x in msg.data: print(x, end=' ')
        # print()


    def get_motor_controllers(self):
        ...


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
