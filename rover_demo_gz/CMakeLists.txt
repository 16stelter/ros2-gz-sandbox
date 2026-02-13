cmake_minimum_required(VERSION 3.8)
project(rover_demo_gz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(rclpy REQUIRED)

# Humble uses Ignition Fortress via ros_ign
find_package(ros_ign_gazebo REQUIRED)
find_package(ros_ign_interfaces REQUIRED)

ament_export_dependencies(std_msgs)
ament_export_dependencies(urdf)
ament_export_dependencies(rclpy)
ament_export_dependencies(ros_ign_gazebo)
ament_export_dependencies(ros_ign_interfaces)

install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_system_plugin_path.dsv")

ament_python_install_package(${PROJECT_NAME})

ament_package()

