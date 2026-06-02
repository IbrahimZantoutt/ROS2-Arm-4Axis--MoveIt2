#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class VisionNode: public rclcpp::Node{
    public:
        VisionNode(): Node("vision_node"){
            // A plain subscription to the Gazebo camera plugin's image topic.
            // The plugin publishes <camera_name>/image_raw, i.e. /camera/image_raw.
            // SensorDataQoS() is best-effort/volatile, which matches sensor streams.
            image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw", rclcpp::SensorDataQoS(),
                std::bind(&VisionNode::imageCallback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Vision node up, listening on /camera/image_raw");
        }

    private:
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg){
            cv::Mat frame;
            try {
                // Convert ROS Image -> OpenCV Mat. The camera publishes RGB (R8G8B8
                // in the urdf), but OpenCV works in BGR, so request bgr8 and cv_bridge
                // handles the channel swap for us.
                frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            } catch (const cv_bridge::Exception & e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
                return;
            }

            // ---- OpenCV processing goes here ----
            // For now, just show it so you can confirm the pipeline works.
            cv::imshow("camera", frame);
            cv::waitKey(1);
        }

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
