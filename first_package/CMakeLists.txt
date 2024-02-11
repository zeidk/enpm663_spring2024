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

#--------------------------------
# C++
#--------------------------------

# set executables

# add dependencies

# install executables to lib/${PROJECT_NAME}

#--------------------------------
# Python
#--------------------------------

# install Python modules from first_package/first_package

# install Python executables

ament_package()
