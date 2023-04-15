from example_interfaces.srv import AddTwoInts
from .stepper_motor import StepperMotor
from .camera import Camera
from .vacuum_pump import Mosfet

import rclpy
from rclpy.node import Node


class ScienceService(Node):

    def __init__(self):
        super().__init__('science_servicer')
        self.srv = self.create_service(AddTwoInts, 'science/science_package', self.add_two_ints_callback)

        self.stepper_motor = StepperMotor()
        self.camera = Camera()
        self.mosfet = Mosfet()

    
    def step_uv_down(self):
        self.stepper_motor.stepUV(True)

    def step_uv_up(self):
        self.stepper_motor.stepUV(False)

    def start_recording(self):
        self.camera.record()

    def stop_recording(self):
        self.camera.close()

    def pump(self, t:float):
        self.mosfet.pump(t)

    def vacuum(self, t:float):
        self.mosfet.vacuum(t)


    def add_two_ints_callback(self, request, response):

        response.sum = 1        # Valid request

        if (request.a == 1):
            print("\n🪜 Lowering Platform for UV 🪜\n")
            self.step_uv_down()

        elif (request.a == 2):
            print("\n🪜 Raising Platform from UV 🪜\n")
            self.step_uv_up()

        elif (request.a == 3):
            print("\n📸🤨 Caught on Camera 🤨📸\n")
            self.start_recording()

        elif (request.a == 4):
            print("\n📷😭 Camera is off 😭\n")
            self.stop_recording()

        elif (request.a == 5):
            print("\n💦😳 💦😳  oh?  💦😳 💦😳\n")
            self.pump(request.b)

        elif (request.a == 6):
            print("\n👄💨 👄💨  OH?  👄💨 👄💨\n")
            self.vacuum(request.b)

        else:
            response.sum = 0    # Invalid request

        self.get_logger().info('Incoming request\nChoice: %d Time: %d s\n' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args)
    service = ScienceService()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()