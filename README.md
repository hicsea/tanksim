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


Here's some info about what I have done to reach this error:
My goal is to modify the turtlesim source code to create obstacles and display them on the GUI. (overall goal is to make this function similarly to autoware, but with a much faster and lightweight program)
I found a git repository online that has the source code for turtlesim in ros2, since the turtlesim source code is not available automatically inside the ros2 environment. 
This repo used Qt4, which is now deprecated... so I began updating the code to use Qt5.
I tried finding a way to just use Qt4, but it seemed easier to just update the code to Qt5 or Qt6.
When I build with the updates, I get the "undefined reference to 'vtable for tanksim::TankFrame'" error. 
From what I have found, it seems this is an error with the use of virtual functions in the code, which in this case would be in the derived class of TankFrame which is derived from a Qt5 class QFrame. All of the code seems pretty straightforward and I can't tell what would be causing it.

