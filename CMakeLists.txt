cmake_minimum_required(VERSION 3.8)
project(bme_common_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate ROS2 message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AutoLog.msg"
  "msg/MavModes.msg"
  "msg/PVT.msg"
  "msg/RELPOSNED.msg"
  "msg/HPPOSLLH.msg"
  "msg/UTMHP.msg"
  "msg/GnssSolution.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

# Install Python scripts
install(PROGRAMS
  scripts/test_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
