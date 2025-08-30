#include "gazebo_utils.hpp"

constexpr const char *__SWAP_SERVICE_NAME = (const char *)"spawn_entity";
constexpr const char *__DELETE_SERVICE_NAME = (const char *)"delete_entity";
constexpr const uint8_t __REQUEST_DELAY_TIME = 5;
constexpr const uint8_t __SERVICE_CHECK_DELAY_TIME = 2;

GazeboUtils::GazeboUtils(rclcpp::Node::SharedPtr node)
    : ptr_gazebo_node_(node)
{
    ptr_spawn_ = node->create_client<gazebo_msgs::srv::SpawnEntity>(__SWAP_SERVICE_NAME);
    ptr_delete_ = node->create_client<gazebo_msgs::srv::DeleteEntity>(__DELETE_SERVICE_NAME);
    ptr_delete_ = node->create_client<gazebo_msgs::srv::DeleteEntity>(__DELETE_SERVICE_NAME);
}

bool GazeboUtils::spawn_model(const std::string &model_name,
                              const std::string &xml,
                              const geometry_msgs::msg::Pose &pose)
{
    if (ptr_spawn_ != NULL && !ptr_spawn_->wait_for_service(std::chrono::seconds(__SERVICE_CHECK_DELAY_TIME)))
    {
        RCLCPP_ERROR(ptr_gazebo_node_->get_logger(), "Service spawn_entity not available");
        return false;
    }
    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name = model_name;
    request->xml = xml;
    request->robot_namespace = model_name;
    request->initial_pose = pose;
    request->reference_frame = "world";

    auto spawnRequestFuture = ptr_spawn_->async_send_request(request);
    auto status = spawnRequestFuture.wait_for(std::chrono::seconds(__REQUEST_DELAY_TIME));

    if (status == std::future_status::ready)
    {
        auto response = spawnRequestFuture.get();
        RCLCPP_INFO(ptr_gazebo_node_->get_logger(), "Spawn response: %s", response->status_message.c_str());
        return (response->success);
    }
    else
    {
        RCLCPP_ERROR(ptr_gazebo_node_->get_logger(), "Failed to call %s service", __SWAP_SERVICE_NAME);
        return false;
    }
}

bool GazeboUtils::delete_model(const std::string &model_name)
{
    if (ptr_delete_ != NULL && !ptr_delete_->wait_for_service(std::chrono::seconds(__SERVICE_CHECK_DELAY_TIME)))
    {
        RCLCPP_ERROR(ptr_gazebo_node_->get_logger(), "Service delete_entity not available");
        return false;
    }
    auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    request->name = model_name;

    auto deleteRequestFuture = ptr_delete_->async_send_request(request);
    auto status = deleteRequestFuture.wait_for(std::chrono::seconds(__REQUEST_DELAY_TIME));

    if (status == std::future_status::ready)
    {
        auto response = deleteRequestFuture.get();
        RCLCPP_INFO(ptr_gazebo_node_->get_logger(), "Delete response: %s", response->status_message.c_str());
        return (response->success);
    }
    else
    {
        RCLCPP_ERROR(ptr_gazebo_node_->get_logger(), "Failed to call %s service", __DELETE_SERVICE_NAME);
        return false;
    }
}
