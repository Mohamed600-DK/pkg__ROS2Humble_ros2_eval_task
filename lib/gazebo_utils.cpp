#include "gazebo_utils.hpp"

constexpr const char *__SWAP_SERVICE_NAME = (const char *)"spawn_entity";
constexpr const char *__DELETE_SERVICE_NAME = (const char *)"delete_entity";
constexpr const char *__GET_MODELS_SERVICE_NAME = (const char *)"get_model_list";
constexpr const uint8_t __REQUEST_DELAY_TIME = 5;
constexpr const uint8_t __SERVICE_CHECK_DELAY_TIME = 2;



GazeboUtils::GazeboUtils(std::shared_ptr<rclcpp::Node> node_)
    : ptr_gazebo_node_(node_)
{
    ptr_spawn_ = ptr_gazebo_node_->create_client<gazebo_msgs::srv::SpawnEntity>(__SWAP_SERVICE_NAME);
    ptr_delete_ = ptr_gazebo_node_->create_client<gazebo_msgs::srv::DeleteEntity>(__DELETE_SERVICE_NAME);
    ptr_get_model_list_ = ptr_gazebo_node_->create_client<gazebo_msgs::srv::GetModelList>(__GET_MODELS_SERVICE_NAME);
}

bool GazeboUtils::priv__check_entity_exists(const std::string &model_name)
{
    auto request = std::make_shared<gazebo_msgs::srv::GetModelList::Request>();

    while (ptr_get_model_list_ != NULL && !ptr_get_model_list_->wait_for_service(std::chrono::seconds(__SERVICE_CHECK_DELAY_TIME)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(ptr_gazebo_node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_WARN(ptr_gazebo_node_->get_logger(), "service not available, waiting again...");
    }

    auto getModelListRequestFuture = ptr_get_model_list_->async_send_request(request);
    auto status = getModelListRequestFuture.wait_for(std::chrono::seconds(__REQUEST_DELAY_TIME));
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(ptr_gazebo_node_, getModelListRequestFuture) == rclcpp::FutureReturnCode::SUCCESS)
    {
        for (const auto &name : getModelListRequestFuture.get()->model_names) {
            if (name == model_name) {
                return true;
            }
        }
    } else {
        RCLCPP_ERROR(ptr_gazebo_node_->get_logger(), "Failed to call service get_model_list");
    }
    return false;
}





bool GazeboUtils::spawn_model(const std::string &model_name,
                              const std::string &xml,
                              const geometry_msgs::msg::Pose &pose)
{
    if (priv__check_entity_exists(model_name))
    {
        RCLCPP_WARN(ptr_gazebo_node_->get_logger(), "Entity %s already exists", model_name.c_str());
        return false;
    }
    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name = model_name;
    request->xml = xml;
    request->robot_namespace = model_name;
    request->initial_pose = pose;
    request->reference_frame = "world";
    

    while (ptr_spawn_ != NULL && !ptr_spawn_->wait_for_service(std::chrono::seconds(__SERVICE_CHECK_DELAY_TIME)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(ptr_gazebo_node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_WARN(ptr_gazebo_node_->get_logger(), "service not available, waiting again...");
    }


    auto spawnRequestFuture = ptr_spawn_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(ptr_gazebo_node_, spawnRequestFuture) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = spawnRequestFuture.get();
        RCLCPP_INFO(ptr_gazebo_node_->get_logger(), "Spawn response: %s", response->status_message.c_str());
        return response->success;
    } else {
        RCLCPP_ERROR(ptr_gazebo_node_->get_logger(), "Failed to call service spawn_entity");
    }
    return false;
}

bool GazeboUtils::delete_model(const std::string &model_name)
{

    auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    request->name = model_name;
    while (ptr_delete_ != NULL && !ptr_delete_->wait_for_service(std::chrono::seconds(__SERVICE_CHECK_DELAY_TIME)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(ptr_gazebo_node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_WARN(ptr_gazebo_node_->get_logger(), "service not available, waiting again...");
    }

    auto deleteRequestFuture = ptr_delete_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(ptr_gazebo_node_, deleteRequestFuture) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = deleteRequestFuture.get();
        RCLCPP_INFO(ptr_gazebo_node_->get_logger(), "Delete response: %s", response->status_message.c_str());
        return response->success;
    } else {
        RCLCPP_ERROR(ptr_gazebo_node_->get_logger(), "Failed to call service delete_entity");
    }
    return false;
}
