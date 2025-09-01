/**
 * @file extend_model_spawner.hpp
 * @brief Extended model spawner with computer vision capabilities
 * @author ros2_eval_task
 * @date 2025
 */

#ifndef EXTEND_MODEL_SPAWNER_HPP
#define EXTEND_MODEL_SPAWNER_HPP

#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "model_spawner.hpp"

/**
 * @class ExtendModelSpawner
 * @brief Extended version of ModelSpawnerNode that captures and saves camera images when models are swapped
 * 
 * This class inherits from ModelSpawnerNode and adds computer vision functionality.
 * It subscribes to a camera topic and automatically captures and saves images whenever
 * a model swap occurs in the simulation. This is useful for creating datasets or
 * monitoring the visual changes in the simulation environment.
 * 
 * The captured images are processed using OpenCV and saved to the package's output directory
 * with descriptive filenames that include the model name and a sequential image ID.
 */
class ExtendModelSpawner : public ModelSpawnerNode
{
private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;     ///< Subscription to camera image topic

  /**
   * @brief Callback function for processing incoming camera images
   * @param msg Shared pointer to the received image message
   * 
   * This callback is triggered whenever a new image is received from the camera topic.
   * It checks if a model swap has recently occurred, and if so:
   * 1. Converts the ROS image message to OpenCV format
   * 2. Applies color space conversion (BGR to RGB)
   * 3. Saves the image to disk with a descriptive filename
   * 
   * The image is only saved when the model_swapped_ flag is true, ensuring that
   * images are captured at the moment of model changes rather than continuously.
   */
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

public:
  /**
   * @brief Constructor for ExtendModelSpawner
   * @param node_name Name for this ROS2 node
   * @param gazebo_client_name Name for the internal Gazebo client node
   * 
   * Initializes the base ModelSpawnerNode functionality and sets up an image
   * subscription to the camera topic. The subscription uses a QoS setting of 1
   * to ensure reliable delivery of image data while maintaining reasonable
   * performance for real-time processing.
   */
  ExtendModelSpawner(const std::string & node_name, const std::string & gazebo_client_name);
};

#endif  // EXTEND_MODEL_SPAWNER_HPP