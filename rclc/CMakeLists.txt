cmake_minimum_required(VERSION 3.5)
set(CMAKE_C_CLANG_TIDY clang-tidy -checks=*)

project(rclc)

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

# pass ROS_DISTRO as preprocessor macro to the compiler
if ($ENV{ROS_DISTRO} STREQUAL eloquent)
  message("Building for ROS 2 Version Eloquent")
  set(ROS_VERSION_ELOQUENT 1)
  # this should be the correct way, but only available from CMake 3.12
  #add_compile_definitions(ROS_VERSION_ELOQUENT)
  add_definitions(-DROS_VERSION_ELOQUENT)
endif()

# pass ROS_DISTRO as preprocessor macro to the compiler
if ($ENV{ROS_DISTRO} STREQUAL dashing)
  message("Building for ROS 2 Version Dashing")
  set(ROS_VERSION_DASHING 1)
  # this should be the correct way, but only available from CMake 3.12
  #add_compile_definitions(ROS_VERSION_DASHING)
  add_definitions(-DROS_VERSION_DASHING)
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

#################################################
# create library
#################################################
add_library(${PROJECT_NAME}
 src/init.c
 src/publisher.c
 src/subscription.c
 src/timer.c
 src/node.c
 src/executor_handle.c
 src/executor.c
 #if (defined ROS_VERSION_ELOQUENT or defined ROS_VERION_DASHING)
   src/rcl_foxy.c
 #endif
 )

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
    test/test_init.cpp
    test/test_node.cpp
    test/test_publisher.cpp
    test/test_subscription.cpp
    test/test_timer.cpp
    test/test_executor_handle.cpp
    test/test_executor.cpp
  )

  target_include_directories(${PROJECT_NAME}_test PRIVATE include)
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
