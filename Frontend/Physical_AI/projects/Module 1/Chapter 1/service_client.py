#!/usr/bin/env python3
# service_client.py - Example ROS 2 service server and client implementation

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ServiceServerNode(Node):
    """
    Service server node that responds to requests from clients.
    Implements an AddTwoInts service that adds two integers.
    """

    def __init__(self):
        super().__init__('service_server')
        # Create a service that listens on the 'add_two_ints' service name
        # The callback function 'add_two_ints_callback' will be called when a request is received
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        # Perform the operation (add the two integers)
        response.sum = request.a + request.b

        # Log the request and response
        self.get_logger().info(f'Incoming request: {request.a} + {request.b}')
        self.get_logger().info(f'Sending response: {response.sum}')

        return response


class ServiceClientNode(Node):
    """
    Service client node that sends requests to the service server.
    """

    def __init__(self):
        super().__init__('service_client')
        # Create a client for the 'add_two_ints' service
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        # Set the request parameters
        self.request.a = a
        self.request.b = b

        # Call the service asynchronously
        self.future = self.cli.call_async(self.request)
        return self.future


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the service server and client nodes
    service_server = ServiceServerNode()
    service_client = ServiceClientNode()

    # Create an executor to handle both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(service_server)
    executor.add_node(service_client)

    # Send a request from the client
    future = service_client.send_request(42, 36)

    try:
        # Spin to handle callbacks
        while rclpy.ok():
            rclpy.spin_once(service_server, timeout_sec=0.1)
            rclpy.spin_once(service_client, timeout_sec=0.1)

            # Check if the response from the service is ready
            if future.done():
                try:
                    response = future.result()
                    service_client.get_logger().info(
                        f'Result of add_two_ints: {response.sum}')
                    break  # Exit after receiving the response
                except Exception as e:
                    service_client.get_logger().info(f'Service call failed: {e}')
                    break

    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        service_server.destroy_node()
        service_client.destroy_node()
        rclpy.shutdown()
        executor.shutdown()


if __name__ == '__main__':
    main()