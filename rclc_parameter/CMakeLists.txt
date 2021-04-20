cmake_minimum_required(VERSION 3.5)
project(rclc_parameter)

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
find_package(ament_cmake REQUIRED)
find_package(rclc REQUIRED)
find_package(rcutils REQUIRED)
find_package(builtin_interfaces REQUIRED)

#################################################
# create library
#################################################
add_library(${PROJECT_NAME} SHARED
  src/rclc_parameter/parameter_server.c
  src/rclc_parameter/parameter.c
  src/rcl_interfaces/msg/set_parameters_result__functions.c
  src/rcl_interfaces/msg/parameter__functions.c
  src/rcl_interfaces/msg/parameter_value__functions.c
  src/rcl_interfaces/msg/list_parameters_result__functions.c
  src/rcl_interfaces/msg/parameter_event__functions.c
  src/rcl_interfaces/string/string_functions.c
  src/rcl_interfaces/srv/get_parameters__functions.c
  src/rcl_interfaces/srv/set_parameters__functions.c
  src/rcl_interfaces/srv/list_parameters__functions.c
  src/rcl_interfaces/srv/get_parameter_types__functions.c
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)

ament_target_dependencies(${PROJECT_NAME}
  rcl
  rclc
  rcutils
  builtin_interfaces
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
# export dependencies
#################################################
# specific order: dependents before dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rcl)
ament_export_dependencies(rcutils)
ament_export_dependencies(builtin_interfaces)
ament_package()
