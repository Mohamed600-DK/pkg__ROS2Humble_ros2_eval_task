cmake_minimum_required(VERSION 3.8)
project(ros2_eval_task)

# ---------------- Dependencies ----------------
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

# === Create Shared Library ===
add_library(gazebo_utils SHARED
  src/gazebo_utils.cpp
)

target_include_directories(gazebo_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gazebo_utils
  rclcpp
  geometry_msgs
  gazebo_msgs
)
# ---------------- Install Python Modules ----------------
ament_python_install_package(${PROJECT_NAME})

# ---------------- Install Launch, RViz, and Worlds ----------------
install(DIRECTORY
  launch
  rviz
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  models/
  DESTINATION $ENV{HOME}/.gazebo/models
)

# ---------------- Install Package.xml ----------------
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
  )
  
# ---------------- Install 'gazebo_utils' library and headers ----------------
install(TARGETS gazebo_utils
  EXPORT export_gazebo_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)
# ---------------- Export ----------------
ament_package()

