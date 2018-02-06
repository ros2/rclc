find_package(rcl_interfaces REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

ament_add_gtest(test_node test_node.cpp)
if(TARGET test_node)
  target_link_libraries(test_node ${PROJECT_NAME})
endif()

ament_add_gtest(test_publisher test_publisher.cpp)
if(TARGET test_publisher)
  target_link_libraries(test_publisher ${PROJECT_NAME})
endif()

ament_add_gtest(test_subscription test_subscription.cpp)
if(TARGET test_subscription)
  target_link_libraries(test_subscription ${PROJECT_NAME})
endif()
