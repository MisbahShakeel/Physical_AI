#!/usr/bin/env python3
# publisher.py - Simple ROS 2 publisher node example

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        # Initialize the node with the name 'minimal_publisher'
        super().__init__('minimal_publisher')

        # Create a publisher that will publish String messages to the 'topic' topic
        # The second parameter (10) is the queue size for the messages
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Create a timer that will call the timer_callback method every 0.5 seconds (500ms)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize a counter to include in the published messages
        self.i = 0

    def timer_callback(self):
        # Create a String message
        msg = String()
        msg.data = f'Hello World: {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message to the console
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.i += 1


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create an instance of the MinimalPublisher node
    minimal_publisher = MinimalPublisher()

    # Start spinning the node so the callback functions are called
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()