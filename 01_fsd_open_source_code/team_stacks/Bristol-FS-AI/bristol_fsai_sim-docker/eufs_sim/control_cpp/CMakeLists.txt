cmake_minimum_required(VERSION 3.8)
project(control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(control_cpp src/control.cpp src/cone.cpp)

ament_target_dependencies(control_cpp rclcpp "eufs_msgs" "ackermann_msgs" visualization_msgs std_srvs
  tf2 tf2_ros geometry_msgs
)

install(TARGETS control_cpp DESTINATION lib/${PROJECT_NAME})

# Add odometry to twist converter
add_executable(odometry_to_twist_converter src/odometry_to_twist_converter.cpp)

ament_target_dependencies(odometry_to_twist_converter rclcpp "eufs_msgs")

install(TARGETS odometry_to_twist_converter DESTINATION lib/${PROJECT_NAME})

ament_package()
