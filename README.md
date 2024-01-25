# tanksim

These instructions assume:
1) You have already sourced the version of ROS2 in your terminal (I used galactic).

Steps to reach error:
1) $ git clone https://github.com/hicsea/tanksim.git
2) $ cd tanksim
3) $ colcon build
5) $ source install/setup.bash
6) $ ros2 run tank_controller tank_controller_node 

Notice the error: 
"from route_initializer.srv import InitializeRoute
ModuleNotFoundError: No module named 'route_initializer.srv'"

