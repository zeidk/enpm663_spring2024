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

# C++
include_directories(include)

add_executable(minimal_node_cpp src/minimal_node.cpp)
add_executable(advanced_node_cpp src/advanced_node.cpp)
add_executable(publisher_node_cpp src/publisher_node.cpp)



# add dependencies
ament_target_dependencies(minimal_node_cpp rclcpp)
ament_target_dependencies(advanced_node_cpp rclcpp)
ament_target_dependencies(publisher_node_cpp rclcpp std_msgs)


# copy executable to lib/${PROJECT_NAME}
install(TARGETS
  minimal_node_cpp
  advanced_node_cpp
  publisher_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Python
# install Python modules
ament_python_install_package(${PROJECT_NAME}
SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# install Python executables
install(PROGRAMS
  script/minimal_node.py
  script/advanced_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
