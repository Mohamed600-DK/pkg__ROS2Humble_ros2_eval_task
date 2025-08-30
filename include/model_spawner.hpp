#ifndef MODEL_SPAWNER_NODE_HPP
#define MODEL_SPAWNER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "gazebo_utils.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <random>
#include <fstream>
#include <sstream>


class ModelSpawnerNode : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::string last_model_;
    std::map<std::string, std::string> model_files;
    std::unique_ptr<GazeboUtils> gazebo_client_;

    void timer_callback();
    double random_range(double min, double max);
public:
    ModelSpawnerNode(std::string node_name,std::string gazebo_client_node_name);
};

#endif // MODEL_SPAWNER_NODE_HPP