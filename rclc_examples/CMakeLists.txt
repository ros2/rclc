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
find_package(std_msgs REQUIRED)


#################################################
# include directories
#################################################
include_directories(include)


#################################################
# examples
#################################################

add_executable(example_let_executor src/example_let_executor.c)
ament_target_dependencies(example_let_executor rcl rclc std_msgs)
#target_include_directories(example_let_executor PRIVATE include)

add_executable(example_let_executor_convenience src/example_let_executor_convenience.c)
ament_target_dependencies(example_let_executor_convenience rcl rclc std_msgs)
#target_include_directories(example_let_executor_convenience PRIVATE include)

install(TARGETS
  example_let_executor
  example_let_executor_convenience
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

