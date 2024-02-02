import rclpy
from rclpy.node import Node
# from geometry_msgs.msg import Pose
from tank_interfaces.srv import InitializeObstacle
import random
from math import pi

class InitializeObstacleService(Node):

    def __init__(self):
        super().__init__('obstacle_initializer_server')
        self.service_ = self.create_service(InitializeObstacle, 'initialize_obstacle', self.initialize_obstacle_callback)
        self.get_logger().info('Initialize Obstacle Service is ready.')

    def initialize_obstacle_callback(self, request, response: InitializeObstacle):

        # Create Obstacle in gui here!
        self.get_logger().info(f'Request received for obstacle with diameter = {request.diameter} at x = {request.x_position} and y = {request.y_position}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = InitializeObstacleService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

