#ifndef GAZEBO_UTILS_HPP
#define GAZEBO_UTILS_HPP

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include "gazebo_msgs/srv/get_model_list.hpp"





class GazeboUtils
{
private:
    std::shared_ptr<rclcpp::Node> ptr_gazebo_node_;
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr ptr_spawn_;
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr ptr_delete_;
    rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedPtr ptr_get_model_list_;

    bool priv__check_entity_exists(const std::string &model_name);
public:
  GazeboUtils(std::string gazebo_client_node_name);

  bool spawn_model(const std::string &model_name,
                   const std::string &xml,
                   const geometry_msgs::msg::Pose &pose);

  bool delete_model(const std::string &model_name);
};

#endif // GAZEBO_UTILS_HPP