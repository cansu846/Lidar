cmake_minimum_required(VERSION 3.8)
project(lidar_process)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)



# find dependencies
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED)


add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
ament_target_dependencies(
  obstacle_avoidance
  rclcpp 
  std_msgs 
  sensor_msgs
  geometry_msgs
  pcl_conversions
)


add_executable(lidar_data src/lidar_data.cpp)
ament_target_dependencies(
  lidar_data
  rclcpp 
  std_msgs 
  sensor_msgs
  pcl_conversions
)

add_executable(lidar_data_planing src/laser_data_planing.cpp)
ament_target_dependencies(
  lidar_data_planing
  rclcpp 
  std_msgs 
  sensor_msgs
  pcl_conversions
)



target_link_libraries( 
  lidar_data
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)
target_link_libraries( 
  obstacle_avoidance
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)
target_link_libraries( 
  lidar_data_planing
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)


include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


install(TARGETS
lidar_data
obstacle_avoidance
lidar_data_planing
  DESTINATION lib/${PROJECT_NAME}
)














#add_executable(lidar_data src/lidar_data.cpp)
#ament_target_dependencies(lidar_data rclcpp sensor_msgs std_msgs)

# install(TARGETS
# lidar_data
#   DESTINATION lib/${PROJECT_NAME})


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
