import rclpy
from rclpy.node import Node
# from geometry_msgs.msg import Pose
from route_initializer.srv import InitializeRoute
import random
from math import pi

class InitializeRouteService(Node):

    def __init__(self):
        super().__init__('route_initializer_server')
        self.service_ = self.create_service(InitializeRoute, 'initialize_route', self.initialize_route_callback)
        self.get_logger().info('Initialize Route Service is ready.')

    def initialize_route_callback(self, request, response: InitializeRoute):
        # Generate random x, y, and theta values
        response.start_x = random.uniform(0,10)
        response.start_y = random.uniform(0,10)
        response.start_theta = random.uniform(-pi,pi)
        response.target_x = random.uniform(0,10)
        response.target_y = random.uniform(0,10)
        response.target_theta = random.uniform(-pi,pi)

        self.get_logger().info(f'Request received. Returning random pose: start_x={response.start_x}, start_y={response.start_y}, start_theta={response.start_theta}')
        self.get_logger().info(f'Request received. Returning random pose: target_x={response.target_x}, target_y={response.target_y}, target_theta={response.target_theta}')

        return response

def main(args=None):
    rclpy.init(args=args)
    node = InitializeRouteService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

