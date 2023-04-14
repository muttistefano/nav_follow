cmake_minimum_required(VERSION 3.5)
project(nav_follow)

add_compile_options(-std=c++17)


find_package(PCL REQUIRED COMPONENTS common io registration)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_srvs REQUIRED)
find_package(generate_parameter_library REQUIRED)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_geometry_msgs
  control_toolbox
  tf2_eigen
  tf2_ros
  pcl_conversions
  std_srvs
)

generate_parameter_library(
  nav_follow_parameters 
  src/nav_follow_parameters.yaml 
)


add_executable(${PROJECT_NAME}_node src/nav_follow.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})


target_link_libraries(${PROJECT_NAME}_node
${PCL_LIBRARIES}
${PCL_LIBRARY_DIRS}
nav_follow_parameters 
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()

