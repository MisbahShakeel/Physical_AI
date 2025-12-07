#!/usr/bin/env python3
# talker_listener.py - Complete ROS 2 talker-listener system with publisher and subscriber

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TalkerNode(Node):
    """
    Talker node that publishes messages to a topic.
    This is similar to the publisher example but as part of a complete system.
    """

    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, this is talker {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Talker: Publishing: "{msg.data}"')
        self.i += 1


class ListenerNode(Node):
    """
    Listener node that subscribes to messages from a topic.
    This is similar to the subscriber example but as part of a complete system.
    """

    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Listener: I heard: "{msg.data}"')


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create both nodes
    talker = TalkerNode()
    listener = ListenerNode()

    # Create an executor to handle both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(talker)
    executor.add_node(listener)

    try:
        # Spin both nodes
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        talker.destroy_node()
        listener.destroy_node()
        rclpy.shutdown()
        executor.shutdown()


if __name__ == '__main__':
    main()