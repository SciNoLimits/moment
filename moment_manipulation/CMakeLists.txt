cmake_minimum_required(VERSION 3.8)
project(moment_manipulation)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moment_interfaces REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)

add_executable(manipulation_server src/manipulation_server.cpp)
target_compile_features(manipulation_server PRIVATE cxx_std_17)
ament_target_dependencies(manipulation_server
  geometry_msgs moment_interfaces moveit_msgs moveit_ros_planning_interface
  rclcpp rclcpp_action tf2_geometry_msgs tf2_ros
)
target_link_libraries(manipulation_server Threads::Threads)

install(TARGETS manipulation_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
