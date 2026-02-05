cmake_minimum_required(VERSION 3.5)
project(EKF-Ball-Predict)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(keisan REQUIRED)
find_package(aruku_interfaces REQUIRED)

# Kompilasi si "Penerjemah" C++
add_executable(ekf_bridge_node src/ekf_bridge.cpp)
target_link_libraries(ekf_bridge_node keisan::keisan)
ament_target_dependencies(ekf_bridge_node rclcpp aruku_interfaces)

# Install script Python simulasi kamu
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/sim.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ekf_bridge_node DESTINATION lib/${PROJECT_NAME})
ament_package()