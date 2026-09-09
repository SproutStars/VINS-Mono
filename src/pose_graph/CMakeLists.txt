cmake_minimum_required(VERSION 3.8)
project(pose_graph)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall -Wno-deprecated-declarations)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_model REQUIRED)
find_package(ros2_compat REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Threads REQUIRED)

add_executable(pose_graph
  src/pose_graph_node.cpp
  src/pose_graph.cpp
  src/keyframe.cpp
  src/utility/CameraPoseVisualization.cpp
  src/ThirdParty/DBoW/BowVector.cpp
  src/ThirdParty/DBoW/FBrief.cpp
  src/ThirdParty/DBoW/FeatureVector.cpp
  src/ThirdParty/DBoW/QueryResults.cpp
  src/ThirdParty/DBoW/ScoringObject.cpp
  src/ThirdParty/DUtils/Random.cpp
  src/ThirdParty/DUtils/Timestamp.cpp
  src/ThirdParty/DVision/BRIEF.cpp
  src/ThirdParty/VocabularyBinary.cpp
)

target_include_directories(pose_graph PRIVATE
  src
  src/utility
  src/ThirdParty/DBoW
  src/ThirdParty/DUtils
  src/ThirdParty/DVision
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(pose_graph
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  camera_model
  ros2_compat
)

target_link_libraries(pose_graph
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  Threads::Threads
)

install(TARGETS pose_graph
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  ../../support_files/brief_k10L6.bin
  ../../support_files/brief_pattern.yml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
