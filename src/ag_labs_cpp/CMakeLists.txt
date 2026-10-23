cmake_minimum_required(VERSION 3.8)
project(ag_labs_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(zero_motor_node src/zero_motor_node.cpp)
ament_target_dependencies(zero_motor_node rclcpp geometry_msgs std_msgs)

# Inverse Perspective Mapping (bird's eye view) node
add_executable(ipm_node src/ipm_node.cpp)
target_include_directories(ipm_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(ipm_node rclcpp sensor_msgs cv_bridge)
target_link_libraries(ipm_node ${OpenCV_LIBS})

# Camera-based lidar (line marking laser scanner)
add_executable(camera_based_lidar_node
  src/camera_based_lidar_main.cpp
  src/camera_based_lidar.cpp
)
target_include_directories(camera_based_lidar_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:src>
)
ament_target_dependencies(camera_based_lidar_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)

install(TARGETS
  zero_motor_node
  ipm_node
  camera_based_lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/)

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
