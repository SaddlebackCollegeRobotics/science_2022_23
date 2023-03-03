
# Ros2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray

# General imports
import threading
import signal

class MinimalPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('funnel_cake_publisher')

        # Specify data type and topic name. Specify queue size (limit amount of queued messages)
        self.publisher_ = self.create_publisher(Int64MultiArray, '/science/funnel_cake', 10)

        # Signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signalHandler)

        # Create a timer that will call the 'timer_callback' function every timer_period second.
        timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        COMMAND_COUNT = 6

        self.msg = Int64MultiArray()
        self.data_list = []

        # Initialize data values
        for i in range(COMMAND_COUNT):
            self.data_list.append(0)

        # Create thread for main loop
        self.thread = threading.Thread(target=self.main_loop)
        self.thread.start()


    # Publisher callback
    def timer_callback(self):

        msg = Int64MultiArray()
        msg.data = 'Hello World: %d' % self.i

        self.publisher_.publish(msg)


    # Callback for Ctrl+C
    def signalHandler(self):
            
        print("\nExited Cleanly")
        quit()


    # Main loop
    def main_loop(self):

        while True:
            
            # Reset data values to zeros
            self.reset_data_values()

            # Get user input
            user_input = input('Enter command: ')

            try:
                if user_input.split(' ')[0] == 'reset':

                    if user_input.split(' ')[1] == 'enc':
                        self.data_list[2] = 1 # Reset encoder values to zero

                    elif user_input.split(' ')[1] == 'pos':
                        self.data_list[3] = 1 # Reset turret position to zero

                elif user_input.split(' ')[0] == 'pos':
                    self.data_list[0] = int(user_input.split(' ')[1]) # Rotation position
                    self.data_list[1] = 1 # Allow rotation

                elif user_input == 'enc':
                    self.data_list[4] = 1 # Print encoder value

                elif user_input == 'stop':
                    self.data_list[5] = 1 # Stop turret motors

                elif user_input == 'help':
                    print('Commands:')
                    print('reset enc: reset encoder')
                    print('reset pos: reset turret position')
                    print('pos: set turret position')
                    print('enc: print encoder value')
                    print('stop: stop turret')
                    print()

                self.msg.data = self.data_list
                self.publisher_.publish(self.msg)
                self.print_data_values()

            except:
                print('Invalid input')
                

    
    def reset_data_values(self):

        for i in range(len(self.data_list)):
            self.data_list[i] = 0

    
    def print_data_values(self):

        for value in self.data_list:
            print(value, end=' ')
        print()


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
