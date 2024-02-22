cmake_minimum_required(VERSION 3.8)
project(av_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(radar_msgs REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# C++

# Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  script/av_sensors_demo.py
  script/av_actions_demo.py
  script/av_cameras_demo.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
