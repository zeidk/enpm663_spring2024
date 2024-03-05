cmake_minimum_required(VERSION 3.8)
project(executor_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------
# Python
# ---------------------------------------------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  script/singlethreaded_executor_exe.py
  script/dual_mutually_exclusive_exe.py
  script/exclusive_reentrant_exe.py
  script/reentrant_exe.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
