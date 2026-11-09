cmake_minimum_required(VERSION 3.8)
project(aeris_safety)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(aeris_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(safety_node
  src/safety_node.cpp
)

target_include_directories(safety_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(safety_node PRIVATE cxx_std_17)

ament_target_dependencies(safety_node
  rclcpp
  px4_msgs
  aeris_interfaces
  std_srvs
)

install(TARGETS safety_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_safety_evaluator test/test_safety_evaluator.cpp)
  target_include_directories(test_safety_evaluator PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()

ament_package()
