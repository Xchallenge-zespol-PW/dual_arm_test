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
  auto left = MoveGroupInterface(node, "left_panda_arm");

  auto right = MoveGroupInterface(node, "right_panda_arm");

  auto both = MoveGroupInterface(node, "both_arms");

  // Set a target Pose
  geometry_msgs::msg::Pose target_left;
  target_left.orientation.x = 0.924;
  target_left.orientation.y = -0.382;
  target_left.orientation.z = 0.0;
  target_left.orientation.w = 0.0;
  target_left.position.x = 0.55;
  target_left.position.y = 0.20;
  target_left.position.z = 0.55;

  geometry_msgs::msg::Pose target_right;
  target_right.orientation.x = 0.924;
  target_right.orientation.y = -0.382;
  target_right.orientation.z = 0.0;
  target_right.orientation.w = 0.0;
  target_right.position.x = 0.35;
  target_right.position.y = -0.20;
  target_right.position.z = 0.25;

  left.setPoseTarget(target_left);
  MoveGroupInterface::Plan plan_left;
  if (!static_cast<bool>(left.plan(plan_left))) {
    RCLCPP_ERROR(logger, "Cant reach goal with left arm");
    rclcpp::shutdown();
    return 1;
  }
  auto const& jt_left = plan_left.trajectory_.joint_trajectory;

  moveit_msgs::msg::RobotState start_right;
  start_right.is_diff = true;
  start_right.joint_state.name = jt_left.joint_names;
  start_right.joint_state.position = jt_left.points.back().positions;
  right.setStartState(start_right);

  right.setPoseTarget(target_right);
  MoveGroupInterface::Plan plan_right;
  if (!static_cast<bool>(right.plan(plan_right))) {
    RCLCPP_ERROR(logger, "Cant reach goal with right arm without colliding with left");
    rclcpp::shutdown();
    return 1;
  }
  auto const& jt_right = plan_right.trajectory_.joint_trajectory;

  std::map<std::string, double> joint_goal;
  for (size_t i = 0; i < jt_left.joint_names.size(); ++i)
    joint_goal[jt_left.joint_names[i]] = jt_left.points.back().positions[i];
  for (size_t i = 0; i < jt_right.joint_names.size(); ++i)
    joint_goal[jt_right.joint_names[i]] = jt_right.points.back().positions[i];

  both.setJointValueTarget(joint_goal);
  both.setPlanningTime(10.0);
  both.setNumPlanningAttempts(5);

  MoveGroupInterface::Plan plan_both;
  if (static_cast<bool>(both.plan(plan_both))) {
    both.execute(plan_both);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}