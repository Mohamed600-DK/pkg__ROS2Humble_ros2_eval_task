# ros2_eval_task

A ROS2 Humble package for dynamic model spawning and image capture in Gazebo simulation environment. This package provides automated battery model management with computer vision integration for robotics evaluation tasks.

## Overview

This package contains three main components:
- **GazeboUtils**: A utility class for interfacing with Gazebo services (spawn, delete, list models)
- **ModelSpawnerNode**: A ROS2 node that periodically spawns random battery models at random positions
- **ExtendModelSpawner**: An extended spawner that captures and saves camera images when models are swapped

### Features

- Automatic spawning and deletion of battery models every 5 seconds
- Random positioning within a defined workspace area
- Camera image capture and saving when model changes occur
- Support for multiple battery model types (9V, Energizer, Varita, LiPo)
- Factory simulation environment with workcell setup

### Package Structure

```
ros2_eval_task/
├── include/                    # Header files
│   ├── gazebo_utils.hpp       # Gazebo service interface utilities
│   ├── model_spawner.hpp      # Basic model spawning node
│   └── extend_model_spawner.hpp # Extended spawner with vision
├── lib/                       # Library implementations
│   └── gazebo_utils.cpp       # Gazebo utilities implementation
├── src/                       # Source files
│   ├── main.cpp              # Main executable entry point
│   ├── model_spawner.cpp     # Model spawner implementation
│   └── extend_model_spawner.cpp # Extended spawner implementation
├── models/                    # Gazebo model definitions
│   ├── battery_9v_leader/     # 9V battery model
│   ├── battery_energizer/     # Energizer battery model
│   ├── battery_varita/        # Varita battery model
│   ├── lipo_battery/          # LiPo battery model
│   ├── workcell/             # Factory workcell environment
│   └── ...                   # Other environment models
├── launch/                    # Launch files
│   └── gazebo.launch.py      # Main Gazebo launch configuration
├── worlds/                    # Gazebo world files
│   └── factory.world         # Factory simulation world
└── out_img/                  # Output directory for captured images
```

## Dependencies (Native)

Make sure you have **ROS 2 Humble** and **Gazebo Classic** installed, then install the required packages:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins

sudo apt install \
  ros-humble-vision-msgs \
  ros-humble-cv-bridge \
  ros-humble-visualization-msgs
```

## Build the Workspace (Native)

```bash
mkdir -p ~/ros2_ws/src
# place this package inside ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install && source install/setup.bash
```

## Quickstart (Native)

```bash
ros2 launch ros2_eval_task gazebo.launch.py
```

---

## Docker

### 1) Build the image
```bash
# from the pkg root (where the Dockerfile is)
docker build -t ros2-gz-classic:humble .
```

### 2) Run the container (with GUI)
```bash
docker run -it --name ros2_gz_gui \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  --device /dev/dri:/dev/dri \
  -v ~/interview_ws:/home/dev/ros2_ws \
  -w /home/dev/ros2_ws \
  ros2-gz-classic:humble
```

### 3) Inside the container: source, build, and launch
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Make Gazebo see your package models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/dev/ros2_ws/src/ros2_eval_task/models

# Build
colcon build --symlink-install

# Overlay
source install/setup.bash

# Launch
ros2 launch ros2_eval_task gazebo.launch.py
```

### 4) Open another shell in the same container (optional)
```bash
docker exec -it ros2_gz_gui bash
```

## Usage

### Quick Start

Launch the complete simulation environment:

```bash
ros2 launch ros2_eval_task gazebo.launch.py
```

This will:
1. Start Gazebo with the factory world
2. Launch the extended model spawner node
3. Begin automatic battery model spawning every 5 seconds
4. Capture and save camera images to `out_img/` directory

### Nodes and Topics

#### Nodes
- `extend_model_spawner_node`: Main node handling model spawning and image capture
- `gazebo_client_node`: Internal client for Gazebo service communication

#### Subscribed Topics
- `/camera/image_raw` (sensor_msgs/Image): Camera feed for image capture

#### Services Used
- `/spawn_entity`: Spawns models in Gazebo
- `/delete_entity`: Removes models from Gazebo  
- `/get_model_list`: Retrieves list of active models

### Configuration

Model spawning parameters can be found in `src/model_spawner.cpp`:
- Timer duration: 5 seconds
- X position range: -0.21 to 0.21 meters
- Y position range: -0.43 to 0.43 meters
- Z position: 1.1 meters (fixed height)

### Output

Images are automatically saved to the `<WORKING_SAPCE>/install/ros2_eval_task/share/ros2_eval_task/out_img/` directory with naming convention:
```
output_image_{model_name}_{image_id}.png
```

## Troubleshooting

### OpenCV Configuration Error
If you encounter OpenCV-related build errors:

```bash
export OpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4
```