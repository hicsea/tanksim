import rclpy
from rclpy.node import Node
# from geometry_msgs.msg import Pose
from obstacle_initializer.srv import InitializeObstacle
import random
from math import pi

class InitializeObstacleService(Node):

    def __init__(self):
        super().__init__('obstacle_initializer_server')
        self.service_ = self.create_service(InitializeObstacle, 'initialize_obstacle', self.initialize_obstacle_callback)
        self.get_logger().info('Initialize Obstacle Service is ready.')

    def initialize_obstacle_callback(self, request, response: InitializeObstacle):


        self.get_logger().info(f'Request received. Returning random pose: start_x={response.start_x}, start_y={response.start_y}, start_theta={response.start_theta}')
        self.get_logger().info(f'Request received. Returning random pose: target_x={response.target_x}, target_y={response.target_y}, target_theta={response.target_theta}')

        return response

def main(args=None):
    rclpy.init(args=args)
    node = InitializeObstacleService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

