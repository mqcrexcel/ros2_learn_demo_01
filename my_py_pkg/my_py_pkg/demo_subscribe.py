#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class RobotReadNews(Node):
    def __init__(self):
        super().__init__("demo_subscribe_node")
        self.subscriber_ = self.create_subscription(
            String, 'robot_news_topic', self.callback_read_robot_news, 10)

    def callback_read_robot_news(self, msg:String):
        self.get_logger().info(f"Subscribe: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotReadNews()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
