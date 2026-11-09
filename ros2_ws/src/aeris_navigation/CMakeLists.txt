cmake_minimum_required(VERSION 3.8)
project(aeris_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(aeris_interfaces REQUIRED)

add_executable(navigation_node
  src/navigation_node.cpp
)

target_include_directories(navigation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(navigation_node PRIVATE cxx_std_17)

ament_target_dependencies(navigation_node
  rclcpp
  sensor_msgs
  aeris_interfaces
)

install(TARGETS navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scan test/test_scan.cpp)
  target_include_directories(test_scan PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_scan sensor_msgs)
endif()

ament_package()
