cmake_minimum_required(VERSION 3.8)
project(px4_reference_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_library(${PROJECT_NAME}_core SHARED
  src/lemniscate_trajectory.cpp
  src/mission_state_machine.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core rclcpp)

add_executable(reference_generator_node src/reference_generator_node.cpp)
target_link_libraries(reference_generator_node ${PROJECT_NAME}_core)
ament_target_dependencies(reference_generator_node rclcpp px4_msgs)

install(TARGETS ${PROJECT_NAME}_core reference_generator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS launch/reference_generator.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(rclcpp)

ament_package()
