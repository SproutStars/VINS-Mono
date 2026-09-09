cmake_minimum_required(VERSION 3.8)
project(benchmark_publisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall -Wno-deprecated-declarations)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_compat REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(benchmark_publisher src/benchmark_publisher_node.cpp)
target_include_directories(benchmark_publisher PRIVATE src ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(benchmark_publisher rclcpp ros2_compat)

install(TARGETS benchmark_publisher DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
