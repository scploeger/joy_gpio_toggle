cmake_minimum_required(VERSION 3.5)
project(joy_gpio_toggle)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
)

add_executable(joy_gpio_toggle_node src/joy_gpio_toggle.cpp)
ament_target_dependencies(joy_gpio_toggle_node rclcpp sensor_msgs)

# Link the wiringPi library
target_link_libraries(joy_gpio_toggle_node wiringPi)

install(TARGETS
  joy_gpio_toggle_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
