cmake_minimum_required(VERSION 3.5)
project(imu_gps_localization)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp_components REQUIRED)


include_directories(
    "include"
    ${PROJECT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
	${std_msgs_INCLUDE_DIRS}
	${sensor_msgs_INCLUDE_DIRS}
	${geometry_msgs_INCLUDE_DIRS}
	${nav_msgs_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${builtin_interfaces_INCLUDE_DIRS}
	${rclcpp_INCLUDE_DIRS}
	${diagnostic_msgs_INCLUDE_DIRS}
	${diagnostic_updater_INCLUDE_DIRS}
	${rclcpp_components_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/imu_gps_localizer/include
	 imu_gps_localizer/include
	 include/imu_gps_localization
)
link_directories(
	${std_msgs_LIBRARY_DIRS}
	${sensor_msgs_LIBRARY_DIRS}
	${geometry_msgs_LIBRARY_DIRS}
	${nav_msgs_LIBRARY_DIRS}
	${EIGEN3_LIBRARY_DIRS}
	${builtin_interfaces_LIBRARY_DIRS}
	${diagnostic_msgs_LIBRARY_DIRS}
	${diagnostic_updater_LIBRARY_DIRS}
	${rclcpp_components_LIBRARY_DIRS}
)
add_subdirectory(imu_gps_localizer)

add_library(ros_wrapper_lib
	src/localization_wrapper.cpp
)
target_link_libraries(ros_wrapper_lib
  ${catkin_LIBRARIES}
  imu_gps_localizer_lib
  glog
)

## Localization node.
add_executable(${PROJECT_NAME}_node src/localization_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ros_wrapper_lib
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  "diagnostic_msgs"
  "diagnostic_updater"
  "geometry_msgs"
  "rclcpp_components"
  ros_wrapper_lib
)


install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
# Install launch rviz config files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

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

ament_package()
