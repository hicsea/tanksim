#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class InterceptorNode(Node):
  def __init__(self):
    super().__init__("interceptor_node")
    self.get_logger().info("Interceptor started!")

    # Publisher and Subscriber
    self.my_pose_sub = self.create_subscription(Twist, "/intercepted_msg", self.pose_callback, 10)
    self.my_interceptor_command = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
  
  def pose_callback(self, msg: Twist):
    self.get_logger().info(f"Current x vel={msg.linear.x} current y vel={msg.linear.y}")
    self.my_interceptor_command.publish(msg)

def main(args=None):
  rclpy.init(args=args)

  #write here
  node = InterceptorNode()
  rclpy.spin(node)
  rclpy.shutdown()


if __name__ == '__main__':
  main()
