cmake_minimum_required(VERSION 3.8)
project(ros2_gz_sandbox)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(rclpy REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)

set(INCLUDE_DIRS ${std_msgs_INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS}
${urdf_INCLUDE_DIRS} ${rclpy_INCLUDE_DIRS} ${gazebo_msgs_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})
link_directories(${LIBRARY_DIRS})

set(LIBS ${std_msgs_LIBRARIES} ${ament_cmake_LIBRARIES} ${urdf_LIBRARIES}
${rclpy_LIBRARIES} ${gazebo_msgs_LIBRARIES})

ament_export_dependencies(std_msgs)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(urdf)
ament_export_dependencies(rclpy)
ament_export_dependencies(gazebo_msgs)
ament_export_include_directories(${INCLUDE_DIRS})

install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_system_plugin_path.dsv")

ament_python_install_package(${PROJECT_NAME})

ament_package()
