cmake_minimum_required(VERSION 3.8)
project(sentinel_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  urdf
  worlds
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/sim_flight_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
