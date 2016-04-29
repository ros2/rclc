cmake_minimum_required(VERSION 2.8.3)

project(rclc)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(
  DIRECTORY include/
  DESTINATION include
)
