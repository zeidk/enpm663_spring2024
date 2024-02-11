cmake_minimum_required(VERSION 3.8)
project(first_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)

#--------------------------------
# C++
#--------------------------------

include_directories(include)

add_executable(minimal_demo_cpp src/minimal_demo.cpp)
add_executable(advanced_demo_cpp src/advanced_demo.cpp)
# add_executable(publisher_demo_cpp src/publisher_demo.cpp)
# add_executable(subscriber_demo_cpp src/subscriber_demo.cpp)

# add dependencies
ament_target_dependencies(minimal_demo_cpp rclcpp)
ament_target_dependencies(advanced_demo_cpp rclcpp)
# ament_target_dependencies(publisher_demo_cpp rclcpp std_msgs)
# ament_target_dependencies(subscriber_demo_cpp rclcpp std_msgs)


# install executables to lib/${PROJECT_NAME}
install(TARGETS
  minimal_demo_cpp
  advanced_demo_cpp
  # publisher_demo_cpp
  # subscriber_demo_cpp
  DESTINATION lib/${PROJECT_NAME}
)

#--------------------------------
# Python
#--------------------------------

# install Python modules from first_package/first_package
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# install Python executables
install(PROGRAMS
  script/minimal_demo.py
  script/advanced_demo.py
  script/publisher_demo.py
  # script/subscriber_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
