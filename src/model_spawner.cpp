/**
 * @file model_spawner.cpp
 * @brief Implementation of the ModelSpawnerNode class for automatic battery model spawning
 * @author ros2_eval_task
 * @date 2025
 */

#include "model_spawner.hpp"
#include "gazebo_utils.hpp"
#include <random>
#include <fstream>
#include <sstream>
#include <memory>
#include <cstdlib>
#include <ctime>

// Configuration constants for model spawning behavior
constexpr  int64_t __TIME_DURATION=5;                          ///< Timer duration in seconds between model swaps
constexpr  double_t __POS_X_MIN=-0.21;                         ///< Minimum X position for model spawning
constexpr  double_t __POS_X_MA=0.21;                           ///< Maximum X position for model spawning
constexpr  double_t __POS_Y_MIN=-0.43;                         ///< Minimum Y position for model spawning
constexpr  double_t __POS_Y_MA=0.43;                           ///< Maximum Y position for model spawning
constexpr  double_t __POS_Z_MI=1.1;                            ///< Minimum Z position for model spawning
constexpr  double_t __POS_Z_MA=1.1;                            ///< Maximum Z position for model spawning

/**
 * @brief List of available battery models for spawning
 * 
 * This static vector contains the names of all battery models that can be
 * randomly selected for spawning. Each name corresponds to a directory
 * in the models/ folder containing the model.sdf file.
 */
static const std::vector<std::string> models_list={
                "battery_9v_leader",
                "battery_energizer",
                "battery_varita",
                "lipo_battery"};


/**
 * @brief Load a file's contents into a string
 * @param path Absolute path to the file to load
 * @return String containing the file's contents
 * @throws std::runtime_error if the file cannot be opened
 * 
 * This utility function reads an entire file into memory as a string.
 * It's used to preload SDF model files for quick access during spawning.
 */
static std::string load_file_to_string(const std::string &path)
{
    std::ifstream file(path);
    if (!file.is_open())
    {
        throw std::runtime_error("Failed to open model file: " + path);
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}
/**
* @brief Generate a random number within the specified range
* @param min Minimum value (inclusive)
* @param max Maximum value (inclusive)
* @return Random double value between min and max
*/
static double random_range(double min, double max)
{
    // Use static random number generator for efficiency
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min, max);
    return dis(gen);
}

ModelSpawnerNode::ModelSpawnerNode(std::string node_name,std::string gazebo_client_node_name) : Node(node_name),model_swapped_(false)
{
    // Initialize random seed for better randomness
    srand(time(nullptr));
    
    // Initialize timer for periodic model swapping (every 5 seconds)
    timer_ = this->create_wall_timer(std::chrono::duration<double>(5.0), std::bind(&ModelSpawnerNode::timer_callback, this));
    
    // Create Gazebo utilities client for model management
    gazebo_client_ = std::make_unique<GazeboUtils>(gazebo_client_node_name);
    
    // Preload all model files into memory for quick access
    for (const auto &model_name : models_list)
    {
        std::string model_path = ament_index_cpp::get_package_share_directory("ros2_eval_task") + "/models/" + model_name + "/model.sdf";
        model_files[model_name] = load_file_to_string(model_path);
    }
    
    // Trigger initial model spawning
    this->timer_callback();
}

std::string ModelSpawnerNode::current_swapped_model()
{
    return last_model_;
}

void ModelSpawnerNode::timer_callback()
{
    // Step 1: Select a random model from the available battery models
    std::string model_name = models_list[rand() % models_list.size()];

    // Step 2: Generate random position within workspace boundaries
    geometry_msgs::msg::Pose pose;
    pose.position.x = random_range(__POS_X_MIN, __POS_X_MA);
    pose.position.y = random_range(__POS_Y_MIN, __POS_Y_MA);
    pose.position.z = random_range(__POS_Z_MI, __POS_Z_MA);

    // Step 3: Get preloaded model XML content (SDF format)
    std::string xml = model_files[model_name];

    // Step 4: Attempt to spawn the new model (deletion is handled internally)
    RCLCPP_INFO(this->get_logger(), "Spawning model: %s", model_name.c_str());
    bool ok = gazebo_client_->spawn_model(model_name, xml, pose);
    if (ok)
    {
        last_model_ = model_name;
        RCLCPP_INFO(this->get_logger(), "Spawned model: %s", model_name.c_str());
        // Set atomic flag to indicate model has been swapped
        this->model_swapped_.store(true);
    }
}



