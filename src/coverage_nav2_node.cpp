#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "action_msgs/msg/goal_status.hpp"

#include "opennav_coverage_msgs/action/navigate_complete_coverage.hpp"

using namespace std::chrono_literals;
using NavigateCoverage = opennav_coverage_msgs::action::NavigateCompleteCoverage;
using GoalHandleCoverage = rclcpp_action::ClientGoalHandle<NavigateCoverage>;

enum class TaskResult
{
  UNKNOWN = 0,
  SUCCEEDED = 1,
  CANCELED = 2,
  FAILED = 3
};

class CoverageNavigatorTester : public rclcpp::Node
{
public:
  CoverageNavigatorTester()
  : Node("coverage_navigator_tester")
  {
    coverage_client_ = rclcpp_action::create_client<NavigateCoverage>(
      this, "navigate_complete_coverage");
  }

  geometry_msgs::msg::Polygon toPolygon(const std::vector<std::array<double, 2>> & field)
  {
    geometry_msgs::msg::Polygon poly;
    for (auto & coord : field) {
      geometry_msgs::msg::Point32 pt;
      pt.x = coord[0];
      pt.y = coord[1];
      poly.points.push_back(pt);
    }
    return poly;
  }

  bool navigateCoverage(const std::vector<std::array<double, 2>> & field)
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for 'NavigateCompleteCoverage' action server...");
    if (!coverage_client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return false;
    }

    auto goal_msg = NavigateCoverage::Goal();
    goal_msg.frame_id = "map";
    goal_msg.polygons.push_back(toPolygon(field));

    RCLCPP_INFO(this->get_logger(), "Sending NavigateCoverage goal...");

    auto send_goal_options = rclcpp_action::Client<NavigateCoverage>::SendGoalOptions();
    send_goal_options.feedback_callback =
      [this](GoalHandleCoverage::SharedPtr, const std::shared_ptr<const NavigateCoverage::Feedback> feedback) {
        this->feedback_ = *feedback;
      };

    auto future_goal_handle = coverage_client_->async_send_goal(goal_msg, send_goal_options);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
      return false;
    }

    goal_handle_ = future_goal_handle.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      return false;
    }

    result_future_ = coverage_client_->async_get_result(goal_handle_);
    return true;
  }

  bool isTaskComplete()
  {
    if (!result_future_.valid()) {
      return true;
    }

    auto ret = rclcpp::spin_until_future_complete(this->get_node_base_interface(),
      result_future_, 100ms);

    if (ret == rclcpp::FutureReturnCode::SUCCESS) {
      auto wrapped_result = result_future_.get();
      status_ = static_cast<int8_t>(wrapped_result.code);
      if (status_ != action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
        RCLCPP_WARN(this->get_logger(), "Task failed with status code: %d", status_);
      } else {
        RCLCPP_INFO(this->get_logger(), "Task succeeded!");
      }
      return true;
    }

    return false; // still running
  }

  NavigateCoverage::Feedback getFeedback()
  {
    return feedback_;
  }

  TaskResult getResult()
  {
    if (status_ == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
      return TaskResult::SUCCEEDED;
    } else if (status_ == action_msgs::msg::GoalStatus::STATUS_ABORTED) {
      return TaskResult::FAILED;
    } else if (status_ == action_msgs::msg::GoalStatus::STATUS_CANCELED) {
      return TaskResult::CANCELED;
    }
    return TaskResult::UNKNOWN;
  }

  void startup(const std::string & node_name = "bt_navigator")
  {
    auto client = this->create_client<lifecycle_msgs::srv::GetState>(node_name + "/get_state");

    RCLCPP_INFO(this->get_logger(), "Waiting for %s to become active...", node_name.c_str());
    while (!client->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Service %s not available, waiting...", (node_name + "/get_state").c_str());
    }

    auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    std::string state = "unknown";

    while (state != "active") {
      auto future = client->async_send_request(req);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS)
      {
        auto resp = future.get();
        state = resp->current_state.label;
        RCLCPP_INFO(this->get_logger(), "Result of get_state: %s", state.c_str());
      }
      std::this_thread::sleep_for(2s);
    }
  }

private:
  rclcpp_action::Client<NavigateCoverage>::SharedPtr coverage_client_;
  GoalHandleCoverage::SharedPtr goal_handle_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr state_client_;

  std::shared_future<GoalHandleCoverage::WrappedResult> result_future_;
  int8_t status_{action_msgs::msg::GoalStatus::STATUS_UNKNOWN};
  NavigateCoverage::Feedback feedback_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CoverageNavigatorTester>();

  node->startup();

  // Example polygon field
  std::vector<std::array<double, 2>> field = {
    {5.0, 5.0}, {5.0, 15.0}, {15.0, 15.0}, {10.0, 5.0}, {5.0, 5.0}
  };

  if (!node->navigateCoverage(field)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to send coverage goal");
    return 1;
  }

  int i = 0;
  while (!node->isTaskComplete() && rclcpp::ok()) {
    auto fb = node->getFeedback();
    if ((i++ % 5) == 0 && fb.estimated_time_remaining.sec > 0) {
      RCLCPP_INFO(node->get_logger(), "Estimated time remaining: %d seconds",
        fb.estimated_time_remaining.sec);
    }
    std::this_thread::sleep_for(1s);
  }

  auto result = node->getResult();
  if (result == TaskResult::SUCCEEDED) {
    RCLCPP_INFO(node->get_logger(), "Goal succeeded!");
  } else if (result == TaskResult::CANCELED) {
    RCLCPP_WARN(node->get_logger(), "Goal was canceled!");
  } else if (result == TaskResult::FAILED) {
    RCLCPP_ERROR(node->get_logger(), "Goal failed!");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Goal returned unknown result!");
  }

  rclcpp::shutdown();
  return 0;
}
