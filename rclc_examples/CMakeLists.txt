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
find_package(example_interfaces REQUIRED)
find_package(rclc_parameter REQUIRED)
find_package(Threads REQUIRED)

include_directories(include)

add_executable(example_executor src/example_executor.c)
target_link_libraries(example_executor rcl::rcl rclc::rclc ${std_msgs_TARGETS})

add_executable(example_executor_only_rcl src/example_executor_only_rcl.c)
target_link_libraries(example_executor_only_rcl rcl::rcl rclc::rclc ${std_msgs_TARGETS})

add_executable(example_executor_trigger src/example_executor_trigger.c)
target_link_libraries(example_executor_trigger rcl::rcl rclc::rclc ${std_msgs_TARGETS})

add_executable(example_service_node src/example_service_node.c)
target_link_libraries(example_service_node rcl::rcl rclc::rclc ${example_interfaces_TARGETS})

add_executable(example_client_node src/example_client_node.c)
target_link_libraries(example_client_node rcl::rcl rclc::rclc ${example_interfaces_TARGETS})

add_executable(example_lifecycle_node src/example_lifecycle_node.c)
target_link_libraries(example_lifecycle_node rcutils::rcutils rcl::rcl rcl_lifecycle::rcl_lifecycle rclc::rclc rclc_lifecycle::rclc_lifecycle ${lifecycle_msgs_TARGETS})

add_executable(example_parameter_server src/example_parameter_server.c)
target_link_libraries(example_parameter_server rclc::rclc rclc_parameter::rclc_parameter)

add_executable(example_sub_context src/example_sub_context.c)
target_link_libraries(example_sub_context rcl::rcl rclc::rclc ${std_msgs_TARGETS})

add_executable(example_pingpong src/example_pingpong.cpp)
target_link_libraries(example_pingpong rcl::rcl rclc::rclc ${std_msgs_TARGETS})

add_executable(example_action_server src/example_action_server.c)
target_link_libraries(example_action_server Threads::Threads)
target_link_libraries(example_action_server rcl::rcl rcl_action::rcl_action rclc::rclc ${example_interfaces_TARGETS})

add_executable(example_action_client src/example_action_client.c)
target_link_libraries(example_action_client rcl::rcl rcl_action::rcl_action rclc::rclc ${example_interfaces_TARGETS})

add_executable(example_short_timer_long_subscription src/example_short_timer_long_subscription.c)
target_link_libraries(example_short_timer_long_subscription rcl::rcl rclc::rclc ${std_msgs_TARGETS})

install(TARGETS
  example_executor
  example_executor_only_rcl
  example_executor_trigger
  example_lifecycle_node
  example_service_node
  example_client_node
  example_parameter_server
  example_sub_context
  example_pingpong
  example_action_server
  example_action_client
  example_short_timer_long_subscription
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

