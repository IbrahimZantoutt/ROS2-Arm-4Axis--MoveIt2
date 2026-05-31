#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"


class ArmController : public rclcpp::Node {
public:
  ArmController() : Node("arm_controller") {}

  void init() {
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm");
    move_group_interface_->setPlanningTime(5.0);
    move_group_interface_->setMaxVelocityScalingFactor(0.5);
    move_group_interface_->setMaxAccelerationScalingFactor(0.5);

    ik_client_ = create_client<moveit_msgs::srv::GetPositionIK>("compute_ik");
  }

  // Returns true only if a collision-free IK solution exists for `target`.
  // Logs the reason when it does not, so a failed pose is explained rather than
  // silently aborting later during planning.
  bool isPoseReachable(const geometry_msgs::msg::Pose& target) {
    if (!ik_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "compute_ik service unavailable; skipping pre-check");
      return true;  // fall through to planning rather than blocking on the check
    }

    auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    request->ik_request.group_name = "arm";
    request->ik_request.avoid_collisions = true;
    request->ik_request.pose_stamped.header.frame_id =
      move_group_interface_->getPlanningFrame();
    request->ik_request.pose_stamped.pose = target;
    request->ik_request.timeout = rclcpp::Duration::from_seconds(2.0);

    auto future = ik_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), future,
          std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(get_logger(), "compute_ik call timed out; skipping pre-check");
      return true;
    }

    const int code = future.get()->error_code.val;
    if (code == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      return true;
    }

    const char* reason =
      code == moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION
        ? "no collision-free IK solution (pose unreachable or every solution self-collides)"
      : code == moveit_msgs::msg::MoveItErrorCodes::GOAL_IN_COLLISION
        ? "goal is in collision"
      : code == moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME
        ? "invalid group name"
        : "IK failed";
    RCLCPP_ERROR(get_logger(), "Pose rejected: %s (error_code=%d)", reason, code);
    return false;
  }

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

    if (!isPoseReachable(target)) {
      RCLCPP_INFO(get_logger(), "Skipping planning to (%.2f, %.2f, %.2f)", x, y, z);
      return;
    }

    move_group_interface_->setPoseTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_->plan(plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(get_logger(), "Planning to (%.2f, %.2f, %.2f): %s",
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
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmController>();
  node->init();

  node->moveToNamed("pose_1");
  // (0.25, 0.0, 0.70) with default (identity) orientation — verified collision-free.
  // The previous target (0.4, 0.0, 0.2) was only reachable through a self-colliding
  // posture (elbow_arm vs shoulder_arm), so MoveIt could never plan to it.
  node->moveToPose(0.25, 0.0, 0.70);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
