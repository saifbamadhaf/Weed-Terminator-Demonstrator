cmake_minimum_required(VERSION 3.5)
project(cartesian_positioner_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(cartesian_positioner_description REQUIRED)
find_package(cartesian_positioner_moveit_config REQUIRED)
find_package(odrive_ros2_control REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
#find_package(moveit_visual_tools)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs)
find_package(detection_interfaces REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(controller_manager_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(plant_tracking src/plant_tracking.cpp)
ament_target_dependencies(plant_tracking moveit control_msgs geometry_msgs detection_interfaces rviz_visual_tools)
install(TARGETS plant_tracking DESTINATION lib/${PROJECT_NAME})

add_executable(positioner_controller src/positioner_controller.cpp)
ament_target_dependencies(positioner_controller moveit moveit_ros_planning_interface control_msgs tf2 tf2_ros geometry_msgs detection_interfaces rviz_visual_tools)
target_include_directories(positioner_controller
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
install(TARGETS positioner_controller DESTINATION lib/${PROJECT_NAME})

add_library(controller_to_servo_node SHARED
   src/joystick_servo.cpp)
rclcpp_components_register_nodes(controller_to_servo_node "cartesian_positioner_controller::JoyToServoPub")
ament_target_dependencies(controller_to_servo_node rclcpp rclcpp_components std_msgs control_msgs sensor_msgs std_srvs controller_manager_msgs detection_interfaces)

ament_export_targets(controller_to_servo_node_export HAS_LIBRARY_TARGET)
ament_export_libraries(controller_to_servo_node)

install(
  TARGETS controller_to_servo_node
  EXPORT controller_to_servo_node_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
