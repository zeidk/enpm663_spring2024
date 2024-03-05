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

# install(DIRECTORY
#   launch
#   config
#   DESTINATION share/${PROJECT_NAME}
# )
# ---------------------------------------------
# C++
# ---------------------------------------------
include_directories(include)

#----------------------------------------------
# Singlethreaded Executor Example
add_executable(singlethreaded_executor_exe src/singlethreaded_executor.cpp)
ament_target_dependencies(singlethreaded_executor_exe rclcpp)

install(TARGETS
singlethreaded_executor_exe
DESTINATION lib/${PROJECT_NAME}
)
#----------------------------------------------
# Multithreaded Executor with 2 Mutually Exclusive Callback Groups
add_executable(dual_mutually_exclusive_exe src/dual_mutually_exclusive.cpp)
ament_target_dependencies(dual_mutually_exclusive_exe rclcpp)

install(TARGETS
dual_mutually_exclusive_exe
DESTINATION lib/${PROJECT_NAME}
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
