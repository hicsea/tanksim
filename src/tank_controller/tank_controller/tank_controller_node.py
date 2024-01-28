#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from route_initializer.srv import InitializeRoute

import os
import signal
from functools import partial

class Controller_Node(Node):
    def __init__(self):
        super().__init__('tank_controller_node')
        # self.get_logger().info("Node Started")
    
        self.route = None
        self.call_initialize_route_service() #initializes the route

        # Publisher and Subscriber
        self.my_pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.my_interceptor_command = self.create_publisher(Twist, "/intercepted_msg", 10)
        # self.my_vel_command = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)


    def pose_callback(self, msg: Pose):
        self.get_logger().info(f"Current x={msg.x} current y={msg.y} and current angle = {msg.theta}")
        # Calculate errors in position
        err_x = self.goal_x - msg.x
        err_y = self.goal_y - msg.y
        err_dist = (err_x**2+err_y**2)**0.5
        
        # Distance error (magnitude of the error vector)
        
        self.get_logger().info(f"Error in x {err_x} and error in y {err_y}")

        # Desired heading based on the position error
        desired_theta = math.atan2(err_y, err_x)
        
        # Error in heading
        err_theta = desired_theta - msg.theta
       
        # Handle wrap-around issues (e.g., if error jumps from +pi to -pi)
        while err_theta > math.pi:
            err_theta -= 2.0 * math.pi
        while err_theta < -math.pi:
            err_theta += 2.0 * math.pi
        self.get_logger().info(f"Desired Angle = {desired_theta} current angle {msg.theta} Error angle {err_theta}")
        # P (ID not required) for linear velocity (distance control)

        Kp_dist = 0.4
            


        # P (ID not required) constants for angular velocity (heading control)
        Kp_theta = 2
        

        # TODO: Add integral and derivative calculations for complete PID

        # PID control for linear velocity
        #l_v = Kp_dist * abs(err_x) # + Ki_dist * integral_dist + Kd_dist * derivative_dist
        l_v = Kp_dist * abs(err_dist) # + Ki_dist * integral_dist + Kd_dist * derivative_dist


        # PID control for angular velocity
        a_v = Kp_theta * err_theta  

        # Send the velocities
        self.my_velocity_cont(l_v, a_v)

        if self.is_goal_reached(msg):
            self.get_logger().info('Goal reached. Terminating Turtlesim.')
            self.terminate_turtlesim()

    def is_goal_reached(self, msg: Pose):
        threshold = 0.01
        distance_to_goal = ((self.goal_x - msg.x)**2 + (self.goal_y - msg.x)**2)**0.5
        return distance_to_goal < threshold
    
    def terminate_turtlesim(self):
        # Terminate the Turtlesim application
        # nodelist = os.system('ros2 node list')
        # self.get_logger().info(nodelist)
        os.system('killall turtlesim_node')
        os.system('killall interceptor_node')
        os.kill(os.getpid(), signal.SIGINT)  # Terminate the current node

    def my_velocity_cont(self, l_v, a_v):
        self.get_logger().info(f"Commanding linear ={l_v} and angular ={a_v}")
        my_msg = Twist()
        my_msg.linear.x = l_v
        my_msg.angular.z = a_v
        self.my_interceptor_command.publish(my_msg)

    def call_initialize_route_service(self):
        client = self.create_client(InitializeRoute, "initialize_route")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("service not available, waiting for service...")
        
        #we can also add in command line handled coordinates using sys.argv[1] and sys.argv[2]
        #we could also read in numbers from a file right here
        request = InitializeRoute.Request()
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_initial_route))



    def callback_set_initial_route(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))
        self.goal_x = response.target_x  # Adjust as needed
        self.goal_y = response.target_y  # Adjust as needed
        

def main(args=None):
    rclpy.init(args=args)
       
    
    node = Controller_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
