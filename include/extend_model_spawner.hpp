#ifndef EXTEND_MODEL_SPAWNER_HPP
#define EXTEND_MODEL_SPAWNER_HPP

#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "model_spawner.hpp"


class ExtendModelSpawner : public ModelSpawnerNode
{
private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
public:
  ExtendModelSpawner(const std::string & node_name, const std::string & gazebo_client_name);

};


#endif  // EXTEND_MODEL_SPAWNER_HPP