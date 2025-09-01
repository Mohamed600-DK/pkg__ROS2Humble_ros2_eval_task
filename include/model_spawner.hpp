/**
 * @file model_spawner.hpp
 * @brief ROS2 node for automatic model spawning in Gazebo simulation
 * @author ros2_eval_task
 * @date 2025
 */

#ifndef MODEL_SPAWNER_NODE_HPP
#define MODEL_SPAWNER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "gazebo_utils.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <random>
#include <fstream>
#include <sstream>

/**
 * @class ModelSpawnerNode
 * @brief A ROS2 node that automatically spawns and swaps battery models in Gazebo at regular intervals
 * 
 * This node manages a collection of battery models and periodically deletes the current model
 * and spawns a new one at a random position within a defined workspace. It serves as a base
 * class for more specialized spawning behaviors.
 * 
 * The node operates on a timer-based system, swapping models every 5 seconds by default.
 * Each new model is positioned randomly within predefined X and Y boundaries at a fixed Z height.
 */
class ModelSpawnerNode : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_;                                    ///< Timer for periodic model swapping
    std::string last_model_;                                                ///< Name of the currently spawned model
    std::map<std::string, std::string> model_files;                        ///< Cache of model SDF file contents
    std::unique_ptr<GazeboUtils> gazebo_client_;                           ///< Utility for Gazebo service interactions

    /**
     * @brief Timer callback function that handles model swapping logic
     * 
     * This function is called periodically by the timer. It:
     * 1. Selects a random model from the available battery models
     * 2. Generates a random position within the workspace boundaries
     * 3. Spawns the new model at the calculated position (deletion is handled automatically)
     * 
     * Note: Model deletion is now handled internally by the spawn_model method for cleaner operation.
     */
    void timer_callback();



protected:
    std::atomic<bool> model_swapped_;                                       ///< Thread-safe flag indicating when a model has been swapped

public:
    /**
     * @brief Constructor for ModelSpawnerNode
     * @param node_name Name for this ROS2 node
     * @param gazebo_client_node_name Name for the internal Gazebo client node
     * 
     * Initializes the node, creates the timer for periodic model swapping,
     * sets up the Gazebo utilities client, and loads all available model files
     * into memory for quick access during spawning operations.
     */
    ModelSpawnerNode(std::string node_name, std::string gazebo_client_node_name);

    /**
     * @brief Get the name of the currently spawned model
     * @return String containing the name of the last successfully spawned model
     * 
     * This method provides access to the name of the model that was most recently
     * spawned in the simulation. Useful for tracking or logging purposes.
     */
    std::string current_swapped_model();
};

#endif // MODEL_SPAWNER_NODE_HPP