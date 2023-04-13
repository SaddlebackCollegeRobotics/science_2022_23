from example_interfaces.srv import AddTwoInts
from ..lib.stepper_motor import StepperMotor
from ..lib.camera import Camera
from ..lib.vacuum_pump import Mosfet

import rclpy
from rclpy.node import Node


class ScienceService(Node):

    def __init__(self):
        super().__init__('science_servicer')
        self.srv = self.create_service(AddTwoInts, 'science/science_package', self.science_package_callback)

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


    def science_package_callback(self, request, response):

        response.sum = 1        # Valid request

        if (request.a == 1):
            self.step_uv_down()
        elif (request.a == 2):
            self.step_uv_up()
        elif (request.a == 3):
            self.start_recording()
        elif (request.a == 4):
            self.stop_recording()
        elif (request.a == 5):
            self.pump(request.b)
        elif (request.a == 6):
            self.vacuum(request.b)
        else:
            response.sum = 0    # Invalid request

        self.get_logger().info('Incoming request\nChoice: %d Time: %d s' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args)
    service = ScienceService()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()