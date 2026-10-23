cmake_minimum_required(VERSION 3.8)
project(ag_map_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Remove conflicting Python package dir created by rosidl_generate_interfaces
# to prevent ament_cmake_python symlink collision on rebuilds
set(PYTHON_PACKAGE_DIR "${CMAKE_BINARY_DIR}/ament_cmake_python/${PROJECT_NAME}/${PROJECT_NAME}")
if(EXISTS "${PYTHON_PACKAGE_DIR}" AND IS_DIRECTORY "${PYTHON_PACKAGE_DIR}")
  file(REMOVE_RECURSE "${PYTHON_PACKAGE_DIR}")
endif()

# Prevent ament_cmake_python symlink on rosidl-generated package
set(ament_cmake_python_SKIP_SYMLINK_INSTALL ON)

# Generate custom service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LoadMap.srv"
  DEPENDENCIES std_srvs
)

# Install textures
install(
  DIRECTORY textures/
  DESTINATION share/${PROJECT_NAME}/textures/
)

# Install SDF models
install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/
)

# Install world files
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds/
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Build map switcher node
find_package(ament_index_cpp REQUIRED)

add_executable(map_switcher_node
  src/map_switcher_node.cpp
)
target_include_directories(map_switcher_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(map_switcher_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  rclcpp::rclcpp
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  ament_index_cpp::ament_index_cpp)

install(TARGETS map_switcher_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Placeholder test - verify texture directory is installed
  ament_add_gtest(test_texture_paths test/test_texture_paths.cpp)
  if(TARGET test_texture_paths)
    target_include_directories(test_texture_paths PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    target_link_libraries(test_texture_paths
      ${PROJECT_NAME}__rosidl_generator_cpp)
  endif()

  ament_add_gtest(test_package_setup test/test_package_setup.cpp)
  if(TARGET test_package_setup)
    target_link_libraries(test_package_setup
      ${PROJECT_NAME}__rosidl_generator_cpp)
  endif()

  ament_add_gtest(test_sdf_models test/test_sdf_models.cpp)
  if(TARGET test_sdf_models)
    target_include_directories(test_sdf_models PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    target_link_libraries(test_sdf_models
      ${PROJECT_NAME}__rosidl_generator_cpp)
  endif()

  ament_add_gtest(test_map_switcher test/test_map_switcher.cpp)
  if(TARGET test_map_switcher)
    target_include_directories(test_map_switcher PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    target_link_libraries(test_map_switcher
      ${PROJECT_NAME}__rosidl_generator_cpp)
  endif()
endif()

ament_package()
