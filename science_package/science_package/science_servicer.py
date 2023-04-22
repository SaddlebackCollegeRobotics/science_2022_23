from example_interfaces.srv import AddTwoInts
from .lib.stepper_motor import StepperMotor
from .lib.vacuum_pump import Mosfet
from .lib.funnel_cake_controller import *

import rclpy
from rclpy.node import Node

import threading


class ScienceService(Node):
    """
    A ROS 2 service that provides different functionality to interact with Science package.

    Attributes
    ----------
    FC_POS : List[int]
        List of integer values representing the positions of the funnel cake.

    Methods
    -------
    __init__()
        Initializes the node, the service and initializes instances of stepper motor, MOSFET and funnel cake controller.
    print_menu()
        Prints the menu options to choose from.
    add_two_ints_callback(request, response)
        The callback function to handle incoming requests and execute the chosen option.
    """

    FC_POS = [-11000, -5275, 250, 5875, 11500]

    def __init__(self):
        """
        Initializes the node, the service and initializes instances of stepper motor, MOSFET and funnel cake controller.
        """
        super().__init__('science_servicer')
        self.srv = self.create_service(AddTwoInts, 'science/science_package', self.add_two_ints_callback)

        self.stepper_motor = StepperMotor()
        self.mosfet = Mosfet()
        self.funnel_cake = Motor_Controller(
            rc=Roboclaw(COMPORT_NAME_1, 115200),
            address=0x80
        )

        print("\n\nScience Package is up\n")
        self.print_menu()

    def print_menu(self):
        """
        Prints the menu options to choose from.
        """
        print("\n\n\n\n\n\n\n\n\n\n\n")
        print("1) Lower Platform\t\t<t=0>\n2) Raise Platform\t\t<t=0>\n3) Pump Request\t\t\t<t=time(s)>\n4) Vacuum Request\t\t<t=time(s)>\n5) Set pos funnel_cake[i]\t<t=index>")

    def add_two_ints_callback(self, request, response):
        """
        The callback function to handle incoming requests and execute the chosen option.

        Parameters
        ----------
        request : AddTwoInts.Request
            The incoming request with two integer values representing the choice and time for that choice.
        response : AddTwoInts.Response
            The response to be sent back to the client with a single integer value representing the status of the request.

        Returns
        -------
        AddTwoInts.Response
            The response with a single integer value representing the status of the request.
        """
        response.sum = 1  # Valid request

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
    """
    Main function that initializes the ScienceService node and starts the ROS2 spin loop.
    """
    rclpy.init(args=args)
    service = ScienceService()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()