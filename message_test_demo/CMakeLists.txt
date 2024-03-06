cmake_minimum_required(VERSION 3.8)
project(message_test_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(interface_demo_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)


# ---------------------------------------------
# C++
# ---------------------------------------------
include_directories(include)

#----------------------------------------------
# Node to test the new message
add_executable(message_test_exe src/message_test_demo.cpp)
ament_target_dependencies(message_test_exe rclcpp interface_demo_msgs)

install(TARGETS
message_test_exe
DESTINATION lib/${PROJECT_NAME}
)
#----------------------------------------------
# Node for the service server
add_executable(service_server_exe src/service_server_demo.cpp)
ament_target_dependencies(service_server_exe rclcpp interface_demo_msgs)

install(TARGETS
service_server_exe
DESTINATION lib/${PROJECT_NAME}
)
#----------------------------------------------
# Node for the service client
add_executable(service_client_exe src/service_client_demo.cpp)
ament_target_dependencies(service_client_exe rclcpp interface_demo_msgs)

install(TARGETS
service_client_exe
DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------
# Python
# ---------------------------------------------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  script/message_test_exe.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
