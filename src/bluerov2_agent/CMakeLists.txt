cmake_minimum_required(VERSION 3.5)
project(bluerov2_agent)

find_package(ament_cmake REQUIRED COMPONENTS)
find_package(ament_cmake_python REQUIRED COMPONENTS)
find_package(rclpy REQUIRED)
find_package(mavros_msgs REQUIRED)

install(PROGRAMS
  bluerov2_agent/bluerov2.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
