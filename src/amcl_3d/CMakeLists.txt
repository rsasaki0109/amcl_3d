cmake_minimum_required(VERSION 3.14)
project(amcl_3d)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS core filters kdtree)

add_definitions(${PCL_DEFINITIONS})

add_executable(amcl_3d_node
  src/node.cpp
  src/mcl.cpp
  src/particle_filter.cpp
  src/xorshift128.cpp
  src/time.cpp
  src/resample_noise/normal_distribution.cpp
  src/measurement_model/lidar_measurement_model.cpp
  src/measurement_model/ndt_pose_measurement_model.cpp
  src/prediction_model/foo_prediction_model.cpp
  src/prediction_model/foo_prediction_model_node.cpp
)

target_compile_features(amcl_3d_node PUBLIC cxx_std_17)

target_include_directories(amcl_3d_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(amcl_3d_node
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

ament_target_dependencies(amcl_3d_node
  builtin_interfaces
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  tf2_sensor_msgs
)

install(TARGETS amcl_3d_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  builtin_interfaces
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  tf2_sensor_msgs
)

ament_package()
