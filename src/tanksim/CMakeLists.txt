cmake_minimum_required(VERSION 3.5)

project(tanksim)

# if(NOT WIN32)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
# endif()
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set (CMAKE_AUTOMOC ON)
set (CMAKE_AUTOUIC ON)
set (CMAKE_AUTORCC ON)
set (CMAKE_INCLUDE_CURRENT_DIR ON)
cmake_policy(SET CMP0071 NEW)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
# find_package(Qt4 4.6 REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)


# include(${QT_USE_FILE})

include_directories(include)
set(tanksim_node_HDRS
  include/tanksim/tank_frame.hpp
  include/tanksim/tank.hpp
)
# qt4_wrap_cpp(tanksim_node_MOCS ${tanksim_node_HDRS})

# set(CMAKE_AUTOMOC ON)



rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Color.msg"
  "msg/Pose.msg"
  "srv/Empty.srv"
  "srv/SetPen.srv"
  "srv/TeleportAbsolute.srv"
  "srv/TeleportRelative.srv"
  "srv/Spawn.srv"
  "srv/Kill.srv"
)

ament_export_dependencies(rosidl_default_runtime)
# MESSAGE("=================${QT_LIBRARIES}======================")


add_executable(tanksim_node 
  src/tanksim.cpp 
  src/tank_frame.cpp 
  src/tank.cpp
  # ${tanksim_node_MOCS}
)

get_available_rmw_implementations(rmw_implementations2)
foreach(rmw_implementation ${rmw_implementations2})
  find_package("${rmw_implementation}" REQUIRED)
  get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")
  foreach(typesupport_impl ${typesupport_impls})
    rosidl_target_interfaces(tanksim_node
      ${PROJECT_NAME} ${typesupport_impl}
    )
  endforeach()
endforeach()

# target_link_libraries(tanksim_node 
#   ${QT_LIBRARIES}
#   ${rclcpp_LIBRARIES}
# )
target_link_libraries(tanksim_node
  Qt5::Widgets
  Qt5::Core 
  ${rclcpp_LIBRARIES})

ament_target_dependencies(tanksim_node
  "rclcpp"
  "std_msgs"
  "geometry_msgs"

)


# add_executable(tank_teleop_key 
#   tutorials/teleop_tank_key.cpp
# )

# foreach(rmw_implementation ${rmw_implementations2})
#   find_package("${rmw_implementation}" REQUIRED)
#   get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")
#   foreach(typesupport_impl ${typesupport_impls})
#     rosidl_target_interfaces(tank_teleop_key
#       ${PROJECT_NAME} ${typesupport_impl}
#     )
#   endforeach()
# endforeach()

# target_link_libraries(tank_teleop_key 
#   ${rclcpp_LIBRARIES}
# )
# ament_target_dependencies(tank_teleop_key
#   "rclcpp"
#   "geometry_msgs"
# )

# install(TARGETS tanksim_node tank_teleop_key
#     DESTINATION bin)

ament_package()