cmake_minimum_required(VERSION 3.8)
project(fabrication_sensor_cpp)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  Boost
  control_msgs
  Eigen3
  interactive_markers
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_servo
  moveit_task_constructor_core
  moveit_visual_tools
  pluginlib
  rclcpp
  rclcpp_action
  rviz_visual_tools
  tf2_geometry_msgs
  geometry_msgs
  tf2_ros
  tf2
  ament_index_cpp
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(flow_sensor_fabrication_demo src/flow_sensor_fabrication_demo.cpp)
add_executable(flow_sensor_fabrication_demo_copy src/flow_sensor_fabrication_demo_copy.cpp)
add_executable(template src/template.cpp)
ament_target_dependencies(flow_sensor_fabrication_demo ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(flow_sensor_fabrication_demo_copy ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(template ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)


target_include_directories(flow_sensor_fabrication_demo PUBLIC include)
target_include_directories(flow_sensor_fabrication_demo_copy PUBLIC include)
target_include_directories(template PUBLIC include)

install(TARGETS flow_sensor_fabrication_demo flow_sensor_fabrication_demo_copy template  DESTINATION lib/${PROJECT_NAME})


ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
