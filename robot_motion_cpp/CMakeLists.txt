cmake_minimum_required(VERSION 3.8)
project(robot_motion_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

find_package(geometric_shapes REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


add_executable(simple_collision src/simple_collision.cpp)


ament_target_dependencies(simple_collision
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
  ros_gz_interfaces
)
target_link_libraries(simple_collision Eigen3::Eigen)


install(TARGETS 
  simple_collision
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY src launch
  DESTINATION share/${PROJECT_NAME}/
)

# install(DIRECTORY include/ DESTINATION include)


ament_package()