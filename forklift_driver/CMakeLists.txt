cmake_minimum_required(VERSION 3.5)
project(forklift_driver)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_auto_find_build_dependencies()
# c++ & python
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# message
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# tf
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
# 3rd party
find_package(serial REQUIRED)

# message
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Meteorcar.msg"
  DEPENDENCIES std_msgs
)

include_directories(include)
add_executable(forklift_drivers node/forklift_drivers.cpp src/stm32.cpp)
ament_target_dependencies(forklift_drivers
 rclcpp 
 std_msgs
 geometry_msgs
 nav_msgs
 sensor_msgs
 tf2_ros
 tf2
 serial
)
add_executable(forklift_control node/forklift_control.cpp)
ament_target_dependencies(forklift_control
 rclcpp 
 std_msgs
 geometry_msgs
 nav_msgs
 sensor_msgs
 tf2_ros
 tf2
 serial
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(forklift_drivers "${cpp_typesupport_target}")
target_link_libraries(forklift_control "${cpp_typesupport_target}")
install(
  TARGETS
  forklift_drivers
  forklift_control
  DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()