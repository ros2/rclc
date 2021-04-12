cmake_minimum_required(VERSION 3.5)
project(rclc_parameter)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- PARAMETER CONFIGURATION ---
set(RCLC_UXRCE_MAX_PARAMETERS "4" CACHE STRING "This value sets the maximum number of parameters for a node")

# Create source files with the defined configuration
configure_file(${PROJECT_SOURCE_DIR}/include/rclc_parameter/config.h.in
  ${PROJECT_SOURCE_DIR}/include/rclc_parameter/config.h)

find_package(ament_cmake REQUIRED)
find_package(rclc REQUIRED)
find_package(rcutils REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_runtime_c REQUIRED)

add_library(${PROJECT_NAME}
  src/rclc_parameter/parameter.c
  src/rclc_parameter/parameter_server.c
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
  rosidl_runtime_c
)

# install
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

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
