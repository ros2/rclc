cmake_minimum_required(VERSION 3.5)

project(rclc)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rosidl_generator_c REQUIRED)
find_package(rosidl_typesupport_test REQUIRED)

ament_export_include_directories(include)
include_directories(include)

set(rclc_sources
  "src/allocators.c"
  "src/convert_rcutils_ret_to_rmw_ret.c"
  "src/error_handling.c"
  "src/names_and_types.c"
  "src/sanity_checks.c"
  "src/node_security_options.c"
  "src/validate_full_topic_name.c"
  "src/validate_namespace.c"
  "src/validate_node_name.c"
)
set_source_files_properties(${rclc_sources} PROPERTIES LANGUAGE "C")
add_library(${PROJECT_NAME} ${rclc_sources})

ament_export_dependencies(
  rcl
  rmw
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
