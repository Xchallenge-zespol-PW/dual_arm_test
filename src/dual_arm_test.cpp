#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "dual_arm_test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("dual_arm_test");

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface_left = MoveGroupInterface(node, "left_panda_arm");

  auto move_group_interface_right = MoveGroupInterface(node, "right_panda_arm");

  // Set a target Pose
  auto const target_pose_left = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 0.0;
    msg.position.x = 0.5;
    msg.position.y = 0.0;
    msg.position.z = 0.0;
    return msg;
  }();
  move_group_interface_left.setPoseTarget(target_pose_left);

  auto const target_pose_right = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 0.0;
    msg.position.x = -0.5;
    msg.position.y = 0.0;
    msg.position.z = 0.0;
    return msg;
  }();
  move_group_interface_right.setPoseTarget(target_pose_right);

  // Create a plan to that target pose
  auto const [success_left, plan_left] = [&move_group_interface_left]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface_left.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Create a plan to that target pose
  auto const [success_right, plan_right] = [&move_group_interface_right]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface_right.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success_left) {
    move_group_interface_left.execute(plan_left);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  if(success_right) {
    move_group_interface_right.execute(plan_right);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}