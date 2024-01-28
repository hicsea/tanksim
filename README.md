# tanksim

These instructions assume:
1) You have already sourced the version of ROS2 in your terminal (I used galactic).

Steps to reach error:
1) $ git clone https://github.com/hicsea/tanksim.git
2) $ cd tanksim
3) $ colcon build

Notice the error: 
stderr: tanksim                                
/bin/ld: CMakeFiles/tanksim_node.dir/src/tank_frame.cpp.o: in function `tanksim::TankFrame::TankFrame(std::shared_ptr<rclcpp::Node>, QWidget*, QFlags<Qt::WindowType>)':
tank_frame.cpp:(.text+0x84): undefined reference to `vtable for tanksim::TankFrame'
/bin/ld: tank_frame.cpp:(.text+0x99): undefined reference to `vtable for tanksim::TankFrame'
/bin/ld: CMakeFiles/tanksim_node.dir/src/tank_frame.cpp.o: in function `tanksim::TankFrame::~TankFrame()':
tank_frame.cpp:(.text+0x6fb): undefined reference to `vtable for tanksim::TankFrame'
/bin/ld: tank_frame.cpp:(.text+0x70d): undefined reference to `vtable for tanksim::TankFrame'
collect2: error: ld returned 1 exit status
make[2]:  [CMakeFiles/tanksim_node.dir/build.make:462: tanksim_node] Error 1
make[1]:  [CMakeFiles/Makefile2:231: CMakeFiles/tanksim_node.dir/all] Error 2
make:  [Makefile:141: all] Error 2

Failed   <<< tanksim [17.2s, exited with code 2]


