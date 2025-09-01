/**
 * @file main.cpp
 * @brief Main entry point for the ros2_eval_task model spawner application
 * @author ros2_eval_task
 * @date 2025
 * 
 * This application creates and runs an ExtendModelSpawner node that automatically
 * spawns battery models in Gazebo simulation and captures camera images when
 * models are swapped. The node operates continuously until shutdown.
 */

#include "extend_model_spawner.hpp"

/**
 * @brief Main function that initializes and runs the ExtendModelSpawner node
 * @param argc Command line argument count
 * @param argv Command line argument values
 * @return Exit status (0 for success)
 * 
 * This function:
 * 1. Initializes the ROS2 system
 * 2. Creates an ExtendModelSpawner node instance
 * 3. Spins the node to process callbacks continuously
 * 4. Cleans up and shuts down when done
 */
int main(int argc, char **argv)
{
    // Initialize ROS2 communication system
    rclcpp::init(argc, argv);
    
    // Create the extended model spawner node
    auto node = std::make_shared<ExtendModelSpawner>("extend_model_spawner_node", "gazebo_client_node");
    
    // Run the node until shutdown signal is received
    rclcpp::spin(node);
    
    // Clean shutdown
    rclcpp::shutdown();
    return 0;
}