cmake_minimum_required(VERSION 3.5)

project(rclc)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rosidl_generator_c REQUIRED)
find_package(rosidl_typesupport_test REQUIRED)

ament_export_include_directories(include)
include_directories(include)

set(rclc_sources
  "src/functions.c"
)
set_source_files_properties(${rclc_sources} PROPERTIES LANGUAGE "C")
add_library(${PROJECT_NAME} ${rclc_sources})
ament_target_dependencies(${PROJECT_NAME}
  rcl
  rosidl_generator_c
)

ament_export_dependencies(
  rcl
  rmw
  rosidl_generator_c
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
