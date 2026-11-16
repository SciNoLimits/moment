cmake_minimum_required(VERSION 3.8)
project(moment_interfaces)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}

  # Actions
  "action/GoToGoal.action"
  "action/Pick.action"
  "action/Place.action"
  # Messages
  # "msg/ObjectInfo.msg"
  # "msg/ObjectPose.msg"

  # Services
  # "srv/DetectObject.srv"

  DEPENDENCIES
    geometry_msgs
    std_msgs
)

# Export runtime dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()
