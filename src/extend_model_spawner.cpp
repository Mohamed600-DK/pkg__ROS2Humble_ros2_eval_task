/**
 * @file extend_model_spawner.cpp
 * @brief Implementation of ExtendModelSpawner class with computer vision capabilities
 * @author ros2_eval_task
 * @date 2025
 */

#include "extend_model_spawner.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#define Subscriber_topic_name "/camera/image_raw"               ///< Camera topic name for image subscription

ExtendModelSpawner::ExtendModelSpawner(const std::string &node_name, const std::string &gazebo_client_name)
    : ModelSpawnerNode(node_name, gazebo_client_name)
{
    // Create subscription to camera topic with QoS setting of 1 for reliable delivery
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        Subscriber_topic_name,
        rclcpp::QoS(1),
        [this](const sensor_msgs::msg::Image::SharedPtr msg)
        {
            this->image_callback(msg);
        });
}

void ExtendModelSpawner::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    static uint64_t image_id=0;                                             ///< Static counter for unique image naming
    
    // Check if a model swap has occurred and reset the flag atomically
    if ( this->model_swapped_.exchange(false) == true)
    {
        // Validate incoming message
        if (!msg)
        {
            RCLCPP_WARN(this->get_logger(), "Received empty image message");
            return;
        }
        
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            // Convert ROS image message to OpenCV format
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        // Process the image using OpenCV
        cv::Mat &image = cv_ptr->image;
        cv::cvtColor(image,image, CV_BGR2RGB);                              // Convert color space from BGR to RGB
        
        // Generate output filename with model name and sequential ID
        std::string image_path = ament_index_cpp::get_package_share_directory("ros2_eval_task") + "/out_img/output_image_" + this->current_swapped_model() + "_" + std::to_string(image_id++) + ".png";
        
        // Save the processed image to disk
        cv::imwrite(image_path, image);
        RCLCPP_INFO(this->get_logger(), "get new image and  saved  at %s", image_path.c_str());
    }
}