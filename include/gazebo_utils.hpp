/**
 * @file gazebo_utils.hpp
 * @brief Utility class for interfacing with Gazebo simulation services
 * @author ros2_eval_task
 * @date 2025
 */

#ifndef GAZEBO_UTILS_HPP
#define GAZEBO_UTILS_HPP

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include "gazebo_msgs/srv/get_model_list.hpp"

/**
 * @class GazeboUtils
 * @brief A utility class that provides convenient methods for interacting with Gazebo simulation services
 * 
 * This class encapsulates the common operations needed to manage models in a Gazebo simulation,
 * including spawning new entities, deleting existing ones, and checking for model existence.
 * It handles the ROS2 service client setup and provides synchronous interfaces for these operations.
 */
class GazeboUtils
{
private:
    std::shared_ptr<rclcpp::Node> ptr_gazebo_node_;                                      ///< Internal ROS2 node for service communication
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr ptr_spawn_;                 ///< Client for spawning entities in Gazebo
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr ptr_delete_;               ///< Client for deleting entities from Gazebo
    rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedPtr ptr_get_model_list_;       ///< Client for retrieving list of models in Gazebo

    /**
     * @brief Check if a model with the given name already exists in the simulation
     * @param model_name The name of the model to check for
     * @return true if the model exists, false otherwise
     */
    bool priv__check_entity_exists(const std::string &model_name);

public:
    /**
     * @brief Constructor for GazeboUtils
     * @param gazebo_client_node_name Name for the internal ROS2 node used for service communication
     * 
     * Initializes the ROS2 node and creates service clients for spawn_entity, delete_entity, 
     * and get_model_list services provided by Gazebo.
     */
    GazeboUtils(std::string gazebo_client_node_name);

    /**
     * @brief Spawn a new model in the Gazebo simulation
     * @param model_name Unique name for the model instance
     * @param xml SDF/URDF XML content describing the model
     * @param pose Initial pose (position and orientation) for the spawned model
     * @return true if the model was successfully spawned, false otherwise
     * 
     * This method will first check if a model with the given name already exists.
     * If it does, the operation will fail and return false. Otherwise, it will
     * attempt to spawn the model at the specified pose.
     */
    bool spawn_model(const std::string &model_name,
                     const std::string &xml,
                     const geometry_msgs::msg::Pose &pose);

    /**
     * @brief Delete a model from the Gazebo simulation
     * @param model_name Name of the model to delete
     * @return true if the model was successfully deleted, false otherwise
     * 
     * Sends a delete request to Gazebo to remove the specified model from the simulation.
     * Returns true on successful deletion, false if the operation failed.
     */
    bool delete_model(const std::string &model_name);
};

#endif // GAZEBO_UTILS_HPP