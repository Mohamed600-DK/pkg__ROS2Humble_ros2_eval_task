/**
 * @file gazebo_utils.cpp
 * @brief Implementation of Gazebo utility functions for model management
 * @author ros2_eval_task
 * @date 2025
 */

#include "gazebo_utils.hpp"

// Service name constants for Gazebo communication
constexpr const char *__SWAP_SERVICE_NAME = (const char *)"spawn_entity";                  ///< Gazebo service name for spawning entities
constexpr const char *__DELETE_SERVICE_NAME = (const char *)"delete_entity";               ///< Gazebo service name for deleting entities  
constexpr const char *__GET_MODELS_SERVICE_NAME = (const char *)"get_model_list";          ///< Gazebo service name for listing models

// Timing constants for service operations
constexpr const uint64_t __SERVICE_CHECK_DELAY_TIME = 200;                                 ///< Delay time (ms) for checking service availability
constexpr const uint64_t __SERVICE_WAITING_DELAY_TIME = 500;                               ///< Delay time (ms) after service operations

GazeboUtils::GazeboUtils(std::string gazebo_client_node_name_)
    : ptr_gazebo_node_(std::make_shared<rclcpp::Node>(gazebo_client_node_name_))
{
    // Create service clients for Gazebo communication
    ptr_spawn_ = ptr_gazebo_node_->create_client<gazebo_msgs::srv::SpawnEntity>(__SWAP_SERVICE_NAME);
    ptr_delete_ = ptr_gazebo_node_->create_client<gazebo_msgs::srv::DeleteEntity>(__DELETE_SERVICE_NAME);
    ptr_get_model_list_ = ptr_gazebo_node_->create_client<gazebo_msgs::srv::GetModelList>(__GET_MODELS_SERVICE_NAME);
}

bool GazeboUtils::priv__check_entity_exists(const std::string &model_name)
{
    // Create request for getting model list
    auto request = std::make_shared<gazebo_msgs::srv::GetModelList::Request>();

    // Wait for service to become available
    while (ptr_get_model_list_ != NULL && !ptr_get_model_list_->wait_for_service(std::chrono::milliseconds(__SERVICE_CHECK_DELAY_TIME*1000)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(ptr_gazebo_node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_WARN(ptr_gazebo_node_->get_logger(), "service not available, waiting again...");
    }

    // Send request and wait for response
    auto getModelListRequestFuture = ptr_get_model_list_->async_send_request(request);
    
    // Check if the request was successful
    if (rclcpp::spin_until_future_complete(ptr_gazebo_node_, getModelListRequestFuture) == rclcpp::FutureReturnCode::SUCCESS)
    {
        // Search for the model name in the returned list
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
    // Check if model already exists to prevent duplicates
    if (priv__check_entity_exists(model_name))
    {
        RCLCPP_WARN(ptr_gazebo_node_->get_logger(), "Entity %s already exists", model_name.c_str());
        return false;
    }
    
    // Create spawn request with model parameters
    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name = model_name;
    request->xml = xml;
    request->robot_namespace = model_name;
    request->initial_pose = pose;
    request->reference_frame = "world";
    
    // Wait for spawn service to become available
    while (ptr_spawn_ != NULL && !ptr_spawn_->wait_for_service(std::chrono::milliseconds(__SERVICE_CHECK_DELAY_TIME*1000)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(ptr_gazebo_node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_WARN(ptr_gazebo_node_->get_logger(), "service not available, waiting again...");
    }

    // Send spawn request and wait for response
    auto spawnRequestFuture = ptr_spawn_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(ptr_gazebo_node_, spawnRequestFuture) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = spawnRequestFuture.get();
        RCLCPP_INFO(ptr_gazebo_node_->get_logger(), "Spawn response: %s", response->status_message.c_str());
        rclcpp::sleep_for(std::chrono::milliseconds(__SERVICE_WAITING_DELAY_TIME));
        return response->success;
    } else {
        RCLCPP_ERROR(ptr_gazebo_node_->get_logger(), "Failed to call service spawn_entity");
    }
    return false;
}

bool GazeboUtils::delete_model(const std::string &model_name)
{
    // Create delete request with model name
    auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    request->name = model_name;
    
    // Wait for delete service to become available
    while (ptr_delete_ != NULL && !ptr_delete_->wait_for_service(std::chrono::milliseconds(__SERVICE_CHECK_DELAY_TIME*1000)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(ptr_gazebo_node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_WARN(ptr_gazebo_node_->get_logger(), "service not available, waiting again...");
    }

    // Send delete request and wait for response
    auto deleteRequestFuture = ptr_delete_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(ptr_gazebo_node_, deleteRequestFuture) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = deleteRequestFuture.get();
        rclcpp::sleep_for(std::chrono::milliseconds(__SERVICE_WAITING_DELAY_TIME));
        RCLCPP_INFO(ptr_gazebo_node_->get_logger(), "Delete response: %s", response->status_message.c_str());
        return response->success;
    } else {
        RCLCPP_ERROR(ptr_gazebo_node_->get_logger(), "Failed to call service delete_entity");
    }
    return false;
}
