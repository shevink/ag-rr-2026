cmake_minimum_required(VERSION 3.8)
project(ag_shared)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch maps assets
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
