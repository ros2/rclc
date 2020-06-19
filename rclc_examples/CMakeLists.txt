cmake_minimum_required(VERSION 3.5)

project(rclc_examples)

#################################################
# compiler settings
#################################################

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(ament_cmake_ros REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_lifecycle REQUIRED)
find_package(rclc REQUIRED)
find_package(rclc_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

include_directories(include)

add_executable(example_executor src/example_executor.c)
ament_target_dependencies(example_executor rcl rclc std_msgs)

add_executable(example_executor_convenience src/example_executor_convenience.c)
ament_target_dependencies(example_executor_convenience rcl rclc std_msgs)

add_executable(example_executor_trigger src/example_executor_trigger.c)
ament_target_dependencies(example_executor_trigger rcl rclc std_msgs)

add_executable(lifecycle_node src/lifecycle_node.c)
ament_target_dependencies(lifecycle_node rcutils rcl rcl_lifecycle rclc rclc_lifecycle lifecycle_msgs)

install(TARGETS
  example_executor
  example_executor_convenience
  example_executor_trigger
  lifecycle_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

