#!/usr/bin/env python3
# subscriber.py - Simple ROS 2 subscriber node example

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        # Initialize the node with the name 'minimal_subscriber'
        super().__init__('minimal_subscriber')

        # Create a subscription that will receive String messages from the 'topic' topic
        # The callback function 'listener_callback' will be called when a message is received
        # The second parameter (10) is the queue size for incoming messages
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

        # Deactivate the subscription to prevent unused variable warnings
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Log the received message to the console
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create an instance of the MinimalSubscriber node
    minimal_subscriber = MinimalSubscriber()

    # Start spinning the node so the callback functions are called
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()