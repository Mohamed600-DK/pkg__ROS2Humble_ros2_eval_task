cmake_minimum_required(VERSION 3.8)
project(ros2_eval_task)

# ---------------- Dependencies ----------------
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# === Create Shared Library ===
add_library(gazebo_utils SHARED
lib/gazebo_utils.cpp
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

# === Create Spawner Node ===
add_executable(model_spawner_node 
    src/main.cpp
    src/model_spawner.cpp
    src/extend_model_spawner.cpp
    )
target_link_libraries(model_spawner_node gazebo_utils)

ament_target_dependencies(model_spawner_node
  rclcpp
  ament_index_cpp
  sensor_msgs
  cv_bridge
  OpenCV

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
LIBRARY DESTINATION lib
)

install(DIRECTORY include/
DESTINATION include
)
# ---------------- Install 'Out_img Directory' ----------------
install(DIRECTORY out_img/
  DESTINATION share/${PROJECT_NAME}/out_img
)

# ---------------- Install 'model_spawner_node' ----------------
install(TARGETS model_spawner_node
  DESTINATION lib/${PROJECT_NAME})
# ---------------- Export ----------------
ament_package()

