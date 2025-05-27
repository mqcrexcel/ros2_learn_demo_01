#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyNode(Node):

    def __init__(self):
        super().__init__("hello_world_node")
        self.counter_ = 0
        self.get_logger().info("Hello world")
        self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello world " + str(self.counter_))
        self.counter_ += 1
        if self.counter_ > 10:
            self.counter_ = 0
            self.get_logger().info("Resetting counter to 0")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()