cmake_minimum_required(VERSION 3.5)

project(rclc)

find_package(ament_cmake_ros REQUIRED)
find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(rosidl_generator_c REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)

ament_export_include_directories(include)

include_directories(include)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

set(${PROJECT_NAME}_SRCS
  src/functions.c
)

include_directories("${CMAKE_CURRENT_BINARY_DIR}/include")

add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS}
)

ament_target_dependencies(${PROJECT_NAME}
  "rcl"
  "rosidl_typesupport_c"
  "rosidl_generator_c"
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "RCLC_BUILDING_DLL")

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(${PROJECT_SOURCE_DIR}/include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rcl
  rosidl_typesupport_c
  rosidl_generator_c
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   add_subdirectory(test)
# endif()

ament_package()

# install(
#   DIRECTORY include/ ${CMAKE_CURRENT_BINARY_DIR}/include/
#   DESTINATION include
# )
