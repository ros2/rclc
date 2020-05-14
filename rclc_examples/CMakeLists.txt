cmake_minimum_required(VERSION 3.5)

project(rclc_examples)

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
find_package(rclc REQUIRED)
find_package(rosidl_generator_c REQUIRED)
find_package(std_msgs REQUIRED)

#################################################
# include directories
#################################################
include_directories(include)

#################################################
# examples
#################################################

add_executable(example_executor src/example_executor.c)
ament_target_dependencies(example_executor rcl rclc rosidl_generator_c std_msgs)

add_executable(example_executor_convenience src/example_executor_convenience.c)
ament_target_dependencies(example_executor_convenience rcl rclc rosidl_generator_c std_msgs)

add_executable(example_executor_trigger src/example_executor_trigger.c)
ament_target_dependencies(example_executor_trigger rcl rclc rosidl_generator_c std_msgs)


install(TARGETS
  example_executor
  example_executor_convenience
  example_executor_trigger
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

