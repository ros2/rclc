cmake_minimum_required(VERSION 2.8.3)

project(rclc)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)

ament_export_include_directories(include)

ament_package()

install(
  DIRECTORY include/
  DESTINATION include
)
