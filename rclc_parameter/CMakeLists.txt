cmake_minimum_required(VERSION 3.5)
project(rclc_parameter)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclc REQUIRED)
find_package(rcutils REQUIRED)
find_package(rcl_interfaces REQUIRED)

add_library(${PROJECT_NAME}
  src/rclc_parameter/parameter.c
  src/rclc_parameter/parameter_client.c
  src/rclc_parameter/parameter_service.c
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclc
  rcutils
  rcl_interfaces
)

# install
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(rcl REQUIRED)
  find_package(rclc REQUIRED)
  find_package(osrf_testing_tools_cpp REQUIRED)
  find_package(std_msgs REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test test/test_parameter.cpp)

  target_include_directories(${PROJECT_NAME}_test PRIVATE include src)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  ament_target_dependencies(${PROJECT_NAME}_test
    rcl
    rcutils
    rosidl_runtime_c
    osrf_testing_tools_cpp
    std_msgs
  )
endif()

# export dependencies
# specific order: dependents before dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rcl)
ament_export_dependencies(rcutils)
ament_export_dependencies(rcl_interfaces)
ament_export_dependencies(rosidl_runtime_c)
ament_package()
