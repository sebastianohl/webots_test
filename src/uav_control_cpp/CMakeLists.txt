cmake_minimum_required(VERSION 3.5)
project(uav_control_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Besides the package specific dependencies we also need the `pluginlib` and `webots_ros2_driver`
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(webots_interfaces REQUIRED)

# The rest is standard ROS 2 packaging description

add_executable(controller   
  src/uav_controller.cpp
)
ament_target_dependencies(controller
  rclcpp 
  sensor_msgs 
  webots_interfaces
)

install(TARGETS
controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()

