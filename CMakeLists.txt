cmake_minimum_required(VERSION 3.8)
project(um982_gnss)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  bondcpp
  sensor_msgs
  mavros_msgs
  nmea_msgs
  gps_msgs
)

find_package(ament_cmake REQUIRED)
foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

include_directories(include)

add_library(tcp SHARED
  src/tcp.cpp)

add_library(serial SHARED
  src/serial.cpp)

add_library(um982_gnss SHARED
  src/um982_gnss.cpp)
target_link_libraries(um982_gnss
  serial)
ament_target_dependencies(um982_gnss
  ${dependencies})
rclcpp_components_register_node(um982_gnss
  PLUGIN "${PROJECT_NAME}::UM982Gnss"
  EXECUTABLE um982_gnss_node)

add_library(ntrip_client SHARED
  src/ntrip_client.cpp)
target_link_libraries(ntrip_client
  tcp)
ament_target_dependencies(ntrip_client
  ${dependencies})
rclcpp_components_register_node(ntrip_client
  PLUGIN "${PROJECT_NAME}::NtripClient"
  EXECUTABLE ntrip_client_node)

install(TARGETS
  tcp
  serial
  um982_gnss
  ntrip_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
