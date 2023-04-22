from example_interfaces.srv import AddTwoInts
from .lib.stepper_motor import StepperMotor
from .lib.vacuum_pump import Mosfet
from .lib.funnel_cake_controller import *

import rclpy
from rclpy.node import Node

import threading



class ScienceService(Node):

    FC_POS = [-11000, -5275, 250, 5875, 11500]

    def __init__(self):
        super().__init__('science_servicer')
        self.srv = self.create_service(AddTwoInts, 'science/science_package', self.add_two_ints_callback)

        self.stepper_motor = StepperMotor()
        self.mosfet = Mosfet()
        self.funnel_cake = Motor_Controller(
            rc = Roboclaw(COMPORT_NAME_1, 115200),
            address = 0x80  
        )

        print("\n\nScience Package is up\n")
        self.print_menu()

    
    def print_menu(self):
        print("\n\n\n\n\n\n\n\n\n\n\n")
        print("1) Lower Platform\t\t<t=0>\n2) Raise Platform\t\t<t=0>\n3) Pump Request\t\t\t<t=time(s)>\n4) Vacuum Request\t\t<t=time(s)>\n5) Set pos funnel_cake[i]\t<t=index>")


    def add_two_ints_callback(self, request, response):

        response.sum = 1        # Valid request

        if (request.a == 1):
            print("\n🪜 Lowering Platform for UV 🪜\n")
            self.stepper_motor.stepUV(True)

        elif (request.a == 2):
            print("\n🪜 Raising Platform from UV 🪜\n")
            self.stepper_motor.stepUV(False)

        elif (request.a == 3):
            print("\n💦😳 💦😳  oh?  💦😳 💦😳\n")
            self.mosfet.pump(request.b)

        elif (request.a == 4):
            print("\n👄💨 👄💨  OH?  👄💨 👄💨\n")
            self.mosfet.vacuum(request.b)

        elif (request.a == 5):  # Set position of funnel cake
            print("Set position funnel_cake[" + "]")
            set_turret_rotation(self.funnel_cake, ScienceService.FC_POS[request.b])

        else:
            response.sum = 0    # Invalid request

        self.get_logger().info('Incoming request\nChoice: %d Time: %d s\n' % (request.a, request.b))
        self.print_menu()

        return response


def main(args=None):
    rclpy.init(args=args)
    service = ScienceService()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()