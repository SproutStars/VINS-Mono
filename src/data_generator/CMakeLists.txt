cmake_minimum_required(VERSION 3.8)
project(data_generator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall -Wno-deprecated-declarations)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(ros2_compat REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(data_generator src/data_generator_node.cpp src/data_generator.cpp)
target_include_directories(data_generator PRIVATE src ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(data_generator rclcpp cv_bridge image_transport ros2_compat)
target_link_libraries(data_generator ${OpenCV_LIBS})

install(TARGETS data_generator DESTINATION lib/${PROJECT_NAME})

ament_package()
