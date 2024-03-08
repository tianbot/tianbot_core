cmake_minimum_required(VERSION 3.8)
project(tianbot_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


set(dependencies
  geometry_msgs
  rclcpp
  std_msgs
  ackermann_msgs
  tf2
  tf2_ros
  nav_msgs
  tf2_geometry_msgs
  sensor_msgs
)

### service files
set(srv_files
  "srv/DebugCmd.srv"
)
rosidl_generate_interfaces(debug_cmd
  ${srv_files}
  LIBRARY_NAME
  tianbot_core
)

ament_export_dependencies(rosidl_default_runtime)
### service files


add_executable(tianbot_core
src/core.cpp 
src/serial.cpp 
src/main.cpp 
src/chassis.cpp 
src/omni.cpp src/ackermann.cpp 
src/differential.cpp src/rover.cpp 
src/protocol.cpp
)

target_include_directories(tianbot_core PUBLIC
  include
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(tianbot_core
  pthread
)


rosidl_get_typesupport_target(cpp_typesupport_target
debug_cmd "rosidl_typesupport_cpp"
)
target_link_libraries(tianbot_core 
${cpp_typesupport_target}
)


ament_target_dependencies(tianbot_core ${dependencies})

# Install 

install(TARGETS tianbot_core
    DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch param scripts
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
