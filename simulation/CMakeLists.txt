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
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/sim_flight_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# The installed flight bridge imports these modules at runtime. Keep them next
# to the executable so the bridge can resolve aerial_node_v2 after colcon install.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../aerial_node_v2/"
  DESTINATION lib/${PROJECT_NAME}/aerial_node_v2
  FILES_MATCHING PATTERN "*.py"
)

ament_package()
