#include "model_spawner.hpp"




int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ModelSpawnerNode>("model_spawner_node", "gazebo_client");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}