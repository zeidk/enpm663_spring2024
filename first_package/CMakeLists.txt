cmake_minimum_required(VERSION 3.8)
project(first_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include/first_package)

add_executable(minimal_node_cpp src/minimal_node.cpp)
ament_target_dependencies(minimal_node_cpp rclcpp)

add_executable(advanced_node_cpp src/advanced_node.cpp)
ament_target_dependencies(advanced_node_cpp rclcpp)

add_executable(publisher_node_cpp src/publisher_node.cpp)
ament_target_dependencies(publisher_node_cpp rclcpp std_msgs)

add_executable(subscriber_node_cpp src/subscriber_node.cpp)
ament_target_dependencies(subscriber_node_cpp rclcpp std_msgs)

install(TARGETS
minimal_node_cpp
advanced_node_cpp
publisher_node_cpp
subscriber_node_cpp
DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME} 
  SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  script/minimal_node.py
  script/advanced_node.py
  script/publisher_node.py
  script/subscriber_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
