cmake_minimum_required(VERSION 3.5)
project(uav_fusion_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Besides the package specific dependencies we also need the `pluginlib` and `webots_ros2_driver`
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(webots_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )


# The rest is standard ROS 2 packaging description

add_executable(fusion
  src/uav_fusion.cpp
)
ament_target_dependencies(fusion
  rclcpp 
  sensor_msgs 
  webots_interfaces
  cv_bridge
)


target_link_libraries(fusion
  ${OpenCV_LIBS}
)

install(TARGETS
fusion
  DESTINATION lib/${PROJECT_NAME})

ament_package()

