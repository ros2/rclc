cmake_minimum_required(VERSION 3.5)
set(CMAKE_C_CLANG_TIDY clang-tidy -checks=*)

project(rclc)

set(CMAKE_VERBOSE_MAKEFILE ON)

#################################################
# compiler settings
#################################################

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_VERBOSE_MAKEFILE ON)

#################################################
# package dependencies
#################################################
find_package(ament_cmake_ros REQUIRED)
find_package(rcl REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosidl_generator_c REQUIRED)
find_package(std_msgs REQUIRED)

if("${rcl_VERSION}" VERSION_LESS "1.0.0")
  message(STATUS "Found rcl version ${rcl_VERSION}, which belongs to Dashing or Eloquent")
  # With CMake 3.12+ use: add_compile_definitions(USE_RCL_WAIT_SET_IS_VALID_BACKPORT)
  add_definitions(-DUSE_RCL_WAIT_SET_IS_VALID_BACKPORT) 
else()
  message(STATUS "Found rcl version ${rcl_VERSION}, which belongs to Foxy or later")
  find_package(rosidl_runtime_c REQUIRED)
endif()

#################################################
# create library
#################################################

add_library(${PROJECT_NAME}
  src/rclc/init.c
  src/rclc/publisher.c
  src/rclc/subscription.c
  src/rclc/timer.c
  src/rclc/node.c
  src/rclc/executor_handle.c
  src/rclc/executor.c
)
if("${rcl_VERSION}" VERSION_LESS "1.0.0")
  target_sources(${PROJECT_NAME} PRIVATE src/rclc/rcl_wait_set_is_valid_backport.c)
endif()

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rcl
  rcutils
  rosidl_generator_c
)


#################################################
# install
#################################################
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

# specific order: dependents before dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

#################################################
# gtest
#################################################
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE) # No copyright header check since link to NOTICE file is not recognized properly.
  find_package(osrf_testing_tools_cpp REQUIRED)
  find_package(std_msgs REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test
    test/rclc/test_init.cpp
    test/rclc/test_node.cpp
    test/rclc/test_publisher.cpp
    test/rclc/test_subscription.cpp
    test/rclc/test_timer.cpp
    test/rclc/test_executor_handle.cpp
    test/rclc/test_executor.cpp
  )

  target_include_directories(${PROJECT_NAME}_test PRIVATE include src)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  ament_target_dependencies(${PROJECT_NAME}_test
    rcl
    rcutils
    rosidl_generator_c
    osrf_testing_tools_cpp
    std_msgs
  )
endif()

#################################################
# export dependencies
#################################################
# specific order: dependents before dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rcl)
ament_export_dependencies(rcutils)
ament_export_dependencies(rosidl_generator_c)
ament_package()
