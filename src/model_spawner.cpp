#include "model_spawner.hpp"
#include "gazebo_utils.hpp"
#include <random>
#include <fstream>
#include <sstream>
#include <memory>

constexpr  int64_t __TIME_DURATION=5;
constexpr  double_t __POS_X_MIN=-0.21;
constexpr  double_t __POS_X_MA=0.21;
constexpr  double_t __POS_Y_MIN=-0.43;
constexpr  double_t __POS_Y_MA=0.43;
constexpr  double_t __POS_Z_MI=1.1;
constexpr  double_t __POS_Z_MA=1.1;



static const std::vector<std::string> models_list={
                "battery_9v_leader",
                "battery_energizer",
                "battery_varita",
                "lipo_battery"};


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

ModelSpawnerNode::ModelSpawnerNode(std::string node_name,std::string gazebo_client_node_name) : Node(node_name)
{
    timer_ = this->create_wall_timer(std::chrono::seconds(__TIME_DURATION), std::bind(&ModelSpawnerNode::timer_callback, this));
    gazebo_client_ = std::make_unique<GazeboUtils>(gazebo_client_node_name);
    
    for (const auto &model_name : models_list)
    {
        std::string model_path = ament_index_cpp::get_package_share_directory("ros2_eval_task") + "/models/" + model_name + "/model.sdf";
        model_files[model_name] = load_file_to_string(model_path);
    }
}

void ModelSpawnerNode::timer_callback()
{
    // 1. Delete previous model (if any)
    if (!last_model_.empty())
    {
        gazebo_client_->delete_model(last_model_);
    }

    // 2. Pick random model
    std::string model_name = models_list[rand() % models_list.size()];

    // 3. Pick random position
    geometry_msgs::msg::Pose pose;
    pose.position.x = random_range(__POS_X_MIN, __POS_X_MA);
    pose.position.y = random_range(__POS_Y_MIN, __POS_Y_MA);
    pose.position.z = random_range(__POS_Z_MI, __POS_Z_MA);

    // 4. get model XML (SDF assumed)
    std::string xml = model_files[model_name];

    // 5. Spawn model
    bool ok = gazebo_client_->spawn_model(model_name, xml, pose);
    if (ok)
    {
        last_model_ = model_name;
        RCLCPP_INFO(this->get_logger(), "Spawned model: %s", model_name.c_str());
        model_swapped_ = true;
    }
}

double ModelSpawnerNode::random_range(double min, double max)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min, max);
    return dis(gen);
}

