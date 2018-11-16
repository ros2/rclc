cmake_minimum_required(VERSION 3.5)
set(CMAKE_C_CLANG_TIDY clang-tidy -checks=*)

project(rclc)

find_package(ament_cmake_ros REQUIRED)
find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(rosidl_generator_c REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)

ament_export_include_directories(include)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

set(${PROJECT_NAME}_SRCS
  src/functions.c
)

add_library(${PROJECT_NAME}
${${PROJECT_NAME}_SRCS}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

ament_target_dependencies(${PROJECT_NAME}
  "rcl"
  "rosidl_typesupport_c"
  "rosidl_generator_c"
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "RCLC_BUILDING_DLL")

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

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)