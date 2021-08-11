cmake_minimum_required(VERSION 3.5)
project(test_rclc)

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)

  # Test rclc example service
  file(TO_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/service_test_expected_output" EXPECTED_OUTPUT)
  configure_file(
      "test/service_test.launch.py.in"
      "test/service_test.launch.py"
      @ONLY
    )
  add_launch_test(
    "${CMAKE_CURRENT_BINARY_DIR}/test/service_test.launch.py"
    TARGET "service_test"
    TIMEOUT 30
    ENV)
endif()

ament_export_include_directories(include)
ament_export_libraries(mode)
ament_export_dependencies(rclc)
ament_export_dependencies(rclc_lifecycle)
ament_export_dependencies(rclc_parameter)
ament_package()
