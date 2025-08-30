# ros2_eval_task

## Dependencies

Make sure you have ROS 2 Humble and Gazebo Classic installed.

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins

sudo apt install \
  ros-humble-vision-msgs \
  ros-humble-cv-bridge \
  ros-humble-visualization-msgs
```

## Build the Workspace

```bash
mkdir -p ~/ros2_ws/src
# place this package inside ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install && source install/setup.bash
```

## Quickstart

```bash
ros2 launch ros2_eval_task gazebo.launch.py
```
## solution for  some errors 
```bash
exportOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4
```