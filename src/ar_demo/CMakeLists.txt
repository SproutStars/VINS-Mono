cmake_minimum_required(VERSION 3.8)
project(ar_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall -Wno-deprecated-declarations)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_model REQUIRED)
find_package(ros2_compat REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(ar_demo_node src/ar_demo_node.cpp)
target_include_directories(ar_demo_node PRIVATE src ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(ar_demo_node rclcpp cv_bridge image_transport camera_model ros2_compat)
target_link_libraries(ar_demo_node ${OpenCV_LIBS})

install(TARGETS ar_demo_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
