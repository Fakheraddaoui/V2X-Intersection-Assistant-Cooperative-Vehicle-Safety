cmake_minimum_required(VERSION 3.8)
project(v2x_conflict_detector)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(v2x_msgs REQUIRED)

add_library(ttc src/ttc.cpp)
target_include_directories(ttc PUBLIC include)

add_executable(conflict_node src/conflict_node.cpp)
target_link_libraries(conflict_node ttc)
ament_target_dependencies(conflict_node rclcpp v2x_msgs)

install(TARGETS conflict_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ttc test/test_ttc.cpp)
  target_link_libraries(test_ttc ttc)
endif()
ament_package()
