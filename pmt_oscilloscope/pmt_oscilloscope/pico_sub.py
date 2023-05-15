# ========================================================================================
# Author:   Jasper Doan     @wluxie
# Date:     05/14/2023
# ========================================================================================


import rclpy                                # ROS2 Python API
from rclpy.node import Node                 # ROS2 Node API
from std_msgs.msg import Float64MultiArray  # ROS2 Float64MultiArray message type
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class PicoSub(Node):
    # '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    # Constructor
    # ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,, 
    def __init__(self):
        super().__init__('pico_sub')

        self.subscription = self.create_subscription(
            Float64MultiArray,
            'science/pico_data',
            self.listener_callback,
            10
        )

        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], lw=2)
        self.ax.set_xlabel('Time (ns)')
        self.ax.set_ylabel('Voltage (mV)')


    # '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    # Listener Callback
    #       Called when a message is received on the 'controls' topic
    # ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
    def listener_callback(self, msg):
        time = np.array(msg.data[0::2])
        voltages = np.array(msg.data[1::2])

        print('Received data:')
        print('\tHighest Voltage: ', np.max(voltages), ' mV')
        print('\tLowest Voltage: ', np.min(voltages), ' mV')

        self.line.set_data(time, voltages)
        self.ax.relim()
        self.ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


    # '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    # Animation
    #       This function is called to animate the plot
    # '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    def animate(self, i):
        return self.line,


    # '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    # Start Animation
    #       This function is called to start the animation
    # '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    def start_animation(self):
        ani = animation.FuncAnimation(self.fig, self.animate, interval=200, blit=True)
        plt.show(block=False)
        plt.pause(0.1)



def main(args=None):
    rclpy.init(args=args)

    pico_sub = PicoSub()

    pico_sub.start_animation()
    rclpy.spin(pico_sub)

    pico_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()