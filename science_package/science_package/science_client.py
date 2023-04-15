import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class ScienceClient(Node):

    def __init__(self):
        super().__init__('science_client')
        self.cli = self.create_client(AddTwoInts, 'science/science_package')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('😭 Science package is not available, waiting again... 😭')

        self.req = AddTwoInts.Request()


    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    science_client = ScienceClient()
    response = science_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    science_client.get_logger().info(
        'Sending request: %d 🤙 %d secs -->' %
        (int(sys.argv[1]), int(sys.argv[2])))
    
    if response == 1:
        science_client.get_logger().info('Result: Success!')
    else:
        science_client.get_logger().info('Result: Invalid request!')

    science_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()