#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <vector>

class VisionNode: public rclcpp::Node{
    public:
        VisionNode(): Node("vision_node"){
            // Create the windows up front and start HighGUI's own event-pump
            // thread. On WSLg/GTK, relying on waitKey(1) alone to both create and
            // map the window is flaky; this makes the windows appear reliably.
            cv::namedWindow("detections", cv::WINDOW_AUTOSIZE);
            cv::namedWindow("mask", cv::WINDOW_AUTOSIZE);
            cv::startWindowThread();

            // A plain subscription to the Gazebo camera plugin's image topic.
            image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw", rclcpp::SensorDataQoS(),
                std::bind(&VisionNode::imageCallback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Vision node up, listening on /camera/image_raw");

            camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                "/camera/camera_info", 10,
                [this](const sensor_msgs::msg::CameraInfo::SharedPtr info) {
                    camera_matrix_ = (cv::Mat_<double>(3,3) <<
                        info->k[0], info->k[1], info->k[2],
                        info->k[3], info->k[4], info->k[5],
                        info->k[6], info->k[7], info->k[8]);
                    dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F); // Gazebo has no distortion
                });
        }

    private:
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg){
            cv::Mat frame;
            try {
                // Convert ROS Image -> OpenCV Mat. Request bgr8 so cv_bridge does
                // the RGB->BGR channel swap for us.
                frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            } catch (const cv_bridge::Exception & e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
                return;
            } catch (const cv::Exception & e) {
                RCLCPP_ERROR(this->get_logger(), "opencv error: %s", e.what());
                return;
            }

            cv::Ptr<cv::aruco::Dictionary> dict =
                cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
            cv::Ptr<cv::aruco::DetectorParameters> params =
                cv::aruco::DetectorParameters::create();

            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(frame, dict, corners, ids, params);

            // Only estimate pose once intrinsics have arrived; camera_matrix_ is
            // empty until the first /camera/camera_info message, and feeding an
            // empty matrix to estimatePoseSingleMarkers throws (outside the
            // try/catch above) and would take the node down.
            if (!ids.empty() && !camera_matrix_.empty()) {
                // estimate 3D pose of each marker
                std::vector<cv::Vec3d> rvecs, tvecs;
                cv::aruco::estimatePoseSingleMarkers(
                    corners, 0.03f,   // 0.03 m = black marker edge in blocks.world
                    camera_matrix_, dist_coeffs_,
                    rvecs, tvecs
                );

                for (size_t i = 0; i < ids.size(); i++) {
                    // tvecs[i] = (x, y, z) in metres relative to camera
                    RCLCPP_INFO(this->get_logger(), "Marker %d at x=%.3f y=%.3f z=%.3f",
                        ids[i], tvecs[i][0], tvecs[i][1], tvecs[i][2]);
                    // Draw the marker's 3D axes so the pose is visible in the window.
                    cv::drawFrameAxes(frame, camera_matrix_, dist_coeffs_,
                                      rvecs[i], tvecs[i], 0.02f);
                }
            }

            // Outline + label any detected markers (drawn even before intrinsics
            // arrive, so you can see detection independently of pose estimation).
            if (!ids.empty()) {
                cv::aruco::drawDetectedMarkers(frame, corners, ids);
            }

            cv::imshow("raw frame", frame);
            cv::waitKey(30);
        }

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}