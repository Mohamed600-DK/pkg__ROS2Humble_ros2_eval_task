#include "extend_model_spawner.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#define Subscriber_topic_name "/camera/image_raw"

ExtendModelSpawner::ExtendModelSpawner(const std::string &node_name, const std::string &gazebo_client_name)
    : ModelSpawnerNode(node_name, gazebo_client_name)
{
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
    static uint64_t image_id=0;
    if ( this->model_swapped_ == true)
    {
        if (!msg)
        {
            RCLCPP_WARN(this->get_logger(), "Received empty image message");
            return;
        }
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        // Process the image using OpenCV
        cv::Mat &image = cv_ptr->image;
        cv::cvtColor(image,image, CV_BGR2RGB);
        std::string image_path = ament_index_cpp::get_package_share_directory("ros2_eval_task") + "/out_img/output_image_" + std::to_string(image_id++) + ".jpg";
        cv::imwrite(image_path, image);
        RCLCPP_INFO(this->get_logger(), "get new image and  saved  at %s", image_path.c_str());
        this->model_swapped_ = false;
    }
}