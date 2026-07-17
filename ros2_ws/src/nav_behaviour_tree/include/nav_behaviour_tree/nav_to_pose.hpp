#ifndef NAV_BEHAVIOUR_TREE__NAV_TO_POSE_HPP_
#define NAV_BEHAVIOUR_TREE__NAV_TO_POSE_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav_behaviour_tree
{

// ---------------------------------------------------------------------------
// NavToPose  --  tag [B-thin]
//
// Thin wrapper around the Nav2 `navigate_to_pose` action. Accepts EITHER:
//   * a ready PoseStamped via the `goal_pose` port (preferred when an upstream
//     node such as ForEachPose / CheckTargetFound produced the pose), OR
//   * x / y / yaw / frame_id scalars (handy for literal poses in the XML).
// `goal_pose` wins if both are present.
//
// StatefulActionNode pattern: onStart() sends the goal, onRunning() polls
// without blocking, onHalted() cancels. Reuse this shape for every other
// action-client leaf (NavThroughPoses, Spin, Dock, ...).
// ---------------------------------------------------------------------------
class NavToPose : public BT::StatefulActionNode
{
public:
  using ActionT = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using Pose = geometry_msgs::msg::PoseStamped;

  NavToPose(
    const std::string & name,
    const BT::NodeConfiguration & config,
    const rclcpp::Node::SharedPtr & node)
  : BT::StatefulActionNode(name, config),
    node_(node)
  {
    client_ = rclcpp_action::create_client<ActionT>(node_, "navigate_to_pose");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Pose>("goal_pose", "ready PoseStamped (preferred)"),
      BT::InputPort<double>("x", "goal x in frame_id [m]"),
      BT::InputPort<double>("y", "goal y in frame_id [m]"),
      BT::InputPort<double>("yaw", 0.0, "goal yaw [rad]"),
      BT::InputPort<std::string>("frame_id", "map", "frame of the goal pose"),
    };
  }

  // bt_main is launched alongside nav2_bringup, so the first tick can land well
  // before the lifecycle manager has activated bt_navigator. Wait long enough to
  // cover a cold nav2 activation rather than racing it.
  static constexpr auto kActionServerTimeout = std::chrono::seconds(30);

  BT::NodeStatus onStart() override
  {
    if (!client_->wait_for_action_server(kActionServerTimeout)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[NavToPose] action server 'navigate_to_pose' not available after %ld s",
        static_cast<long>(kActionServerTimeout.count()));
      return BT::NodeStatus::FAILURE;
    }

    ActionT::Goal goal;
    Pose goal_pose;
    if (getInput("goal_pose", goal_pose) && !goal_pose.header.frame_id.empty()) {
      goal.pose = goal_pose;                       // use the supplied pose as-is
    } else {
      double x = 0.0, y = 0.0, yaw = 0.0;
      std::string frame = "map";
      if (!getInput("x", x) || !getInput("y", y)) {
        RCLCPP_ERROR(node_->get_logger(), "[NavToPose] need goal_pose OR x+y");
        return BT::NodeStatus::FAILURE;
      }
      getInput("yaw", yaw);
      getInput("frame_id", frame);
      goal.pose.header.frame_id = frame;
      goal.pose.pose.position.x = x;
      goal.pose.pose.position.y = y;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      goal.pose.pose.orientation = tf2::toMsg(q);
    }
    goal.pose.header.stamp = node_->now();

    done_ = false;
    accepted_ = false;
    result_code_ = rclcpp_action::ResultCode::UNKNOWN;

    rclcpp_action::Client<ActionT>::SendGoalOptions opts;
    opts.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        done_ = true;
        result_code_ = result.code;
      };
    future_goal_handle_ = client_->async_send_goal(goal, opts);

    RCLCPP_INFO(
      node_->get_logger(), "[NavToPose] goal (%.2f, %.2f) in '%s'",
      goal.pose.pose.position.x, goal.pose.pose.position.y,
      goal.pose.header.frame_id.c_str());
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (!accepted_) {
      if (future_goal_handle_.wait_for(std::chrono::seconds(0)) ==
        std::future_status::ready)
      {
        goal_handle_ = future_goal_handle_.get();
        if (!goal_handle_) {
          RCLCPP_ERROR(node_->get_logger(), "[NavToPose] goal rejected");
          return BT::NodeStatus::FAILURE;
        }
        accepted_ = true;
      } else {
        return BT::NodeStatus::RUNNING;
      }
    }
    if (done_) {
      if (result_code_ == rclcpp_action::ResultCode::SUCCEEDED) {
        return BT::NodeStatus::SUCCESS;
      }
      RCLCPP_WARN(node_->get_logger(), "[NavToPose] navigation did not succeed");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    if (accepted_ && goal_handle_) {
      client_->async_cancel_goal(goal_handle_);
      RCLCPP_INFO(node_->get_logger(), "[NavToPose] halted -> cancelling goal");
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ActionT>::SharedPtr client_;
  std::shared_future<GoalHandle::SharedPtr> future_goal_handle_;
  GoalHandle::SharedPtr goal_handle_;

  bool done_{false};
  bool accepted_{false};
  rclcpp_action::ResultCode result_code_{rclcpp_action::ResultCode::UNKNOWN};
};

}  // namespace nav_behaviour_tree

#endif  // NAV_BEHAVIOUR_TREE__NAV_TO_POSE_HPP_