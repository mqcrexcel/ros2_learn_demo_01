#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String  
# Import the message type for publishing (String)
# Note: Ensure that 'example_interfaces' is installed in your ROS 2 environment


class RobotNewsNode(Node):
    def __init__(self):
        super().__init__("demo_publish_node")
        self.publisher_ = self.create_publisher(String, 'robot_news_topic', 10)
        self.counter_value_ = 0

        # Create a timer to call the publish_robot_news method every 0.5 seconds
        self.timer_ = self.create_timer(0.5, self.publish_robot_news)
        self.get_logger().info(f"RobotNewsNode has been started")

    # Method name publish_robot_news
    def publish_robot_news(self):
        msg = String()
        msg.data = f"Robot News topic with value: {self.counter_value_}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: '{msg.data}'")
        self.counter_value_ += 1
        if self.counter_value_ > 100:
            self.counter_value_ = 0
            self.get_logger().info("Resetting counter to 0")


def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
