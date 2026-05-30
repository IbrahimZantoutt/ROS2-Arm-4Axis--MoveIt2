#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/pose.hpp"


class ArmController : public rclcpp::Node {
public:
  ArmController() : Node("arm_controller") {}

  void init() {
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm");
    move_group_interface_->setPlanningTime(5.0);
    move_group_interface_->setMaxVelocityScalingFactor(0.5);
    move_group_interface_->setMaxAccelerationScalingFactor(0.5);
  }

  void moveToXYZ(double x, double y, double z) {
    move_group_interface_->setPositionTarget(x, y, z);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_->plan(plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(get_logger(), "Planning to (%.2f, %.2f, %.2f): %s",
      x, y, z, success ? "SUCCEEDED" : "FAILED");

    if (success) {
      move_group_interface_->execute(plan);
    }
  }

  // For 6-axis arms only — requires full 6-DOF to solve position + orientation
  void moveToPose(double x, double y, double z,
                  double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0) {
    geometry_msgs::msg::Pose target;
    target.position.x = x;
    target.position.y = y;
    target.position.z = z;
    target.orientation.x = qx;
    target.orientation.y = qy;
    target.orientation.z = qz;
    target.orientation.w = qw;

    move_group_interface_->setPoseTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_->plan(plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(get_logger(), "Planning to pose (%.2f, %.2f, %.2f): %s",
      x, y, z, success ? "SUCCEEDED" : "FAILED");

    if (success) {
      move_group_interface_->execute(plan);
    }
  }

  void moveToNamed(const std::string& pose_name) {
    move_group_interface_->setNamedTarget(pose_name);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_->plan(plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(get_logger(), "Planning to '%s': %s",
      pose_name.c_str(), success ? "SUCCEEDED" : "FAILED");

    if (success) {
      move_group_interface_->execute(plan);
    }
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmController>();
  node->init();

  for (const auto& pose : {"home", "pose_1", "pose_2", "pose_3", "pose_4", "pose_5"}) {
    node->moveToNamed(pose);
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
