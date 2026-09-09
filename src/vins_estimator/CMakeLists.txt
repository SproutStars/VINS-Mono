cmake_minimum_required(VERSION 3.8)
project(vins_estimator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall -Wno-deprecated-declarations)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_model REQUIRED)
find_package(ros2_compat REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Threads REQUIRED)

add_executable(vins_estimator
  src/estimator_node.cpp
  src/parameters.cpp
  src/estimator.cpp
  src/feature_manager.cpp
  src/factor/pose_local_parameterization.cpp
  src/factor/projection_factor.cpp
  src/factor/projection_td_factor.cpp
  src/factor/marginalization_factor.cpp
  src/utility/utility.cpp
  src/utility/visualization.cpp
  src/utility/CameraPoseVisualization.cpp
  src/initial/solve_5pts.cpp
  src/initial/initial_aligment.cpp
  src/initial/initial_sfm.cpp
  src/initial/initial_ex_rotation.cpp
)

target_include_directories(vins_estimator PRIVATE
  src
  src/utility
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(vins_estimator
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  cv_bridge
  camera_model
  ros2_compat
)

target_link_libraries(vins_estimator
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  Threads::Threads
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS vins_estimator
  DESTINATION lib/${PROJECT_NAME}
)

# Configuration and visualization resources used by the launch files.
# install(DIRECTORY ./config/
#   DESTINATION share/${PROJECT_NAME}/config
# )

# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
