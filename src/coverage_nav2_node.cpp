#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "action_msgs/msg/goal_status.hpp"

#include "opennav_coverage_msgs/action/navigate_complete_coverage.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;
using NavigateCoverage = opennav_coverage_msgs::action::NavigateCompleteCoverage;
using GoalHandleCoverage = rclcpp_action::ClientGoalHandle<NavigateCoverage>;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandlePose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class CoverageNavigatorTester : public rclcpp::Node
{
public:
  CoverageNavigatorTester()
  : Node("coverage_navigator_tester")
  {
    coverage_client_ = rclcpp_action::create_client<NavigateCoverage>(this, "navigate_complete_coverage");
    nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    coverage_sub_ = this->create_subscription<geometry_msgs::msg::Polygon>(
      "/coverage_start", 10, std::bind(&CoverageNavigatorTester::coverageCallback, this, std::placeholders::_1));

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    first_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/coverage_first_pose", qos,
      std::bind(&CoverageNavigatorTester::firstPoseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Coverage tester node ready. Waiting for polygon on /coverage_zone.");
  }

private:
  // --- Funzioni di utilità ---
  geometry_msgs::msg::Polygon toPolygon(const std::vector<std::array<double, 2>> & field)
  {
    geometry_msgs::msg::Polygon poly;
    for (const auto & coord : field) {
      geometry_msgs::msg::Point32 pt;
      pt.x = coord[0];
      pt.y = coord[1];
      poly.points.push_back(pt);
    }
    return poly;
  }

  void coverageCallback(const geometry_msgs::msg::Polygon::SharedPtr msg)
  {
    if (is_task_running_) {
      RCLCPP_WARN(this->get_logger(), "A task is already running. Ignoring coverage message.");
      return;
    }

    if (msg->points.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty coverage polygon, ignoring.");
      return;
    }

    field.clear();
    for (const auto &pt : msg->points) {
      field.push_back({pt.x, pt.y});
    }

    RCLCPP_INFO(this->get_logger(), "Received coverage polygon with %zu points. Starting coverage directly.", field.size());
    is_task_running_ = true;

    numAttempts = 0;
    startCoverage();
  }

  void firstPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    auto fixed_pose = *msg;
    fixed_pose.header.frame_id = "map";
    first_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(fixed_pose);
    RCLCPP_INFO(this->get_logger(), "Received start pose: x=%.2f, y=%.2f (frame=%s)",
                fixed_pose.pose.position.x, fixed_pose.pose.position.y,
                fixed_pose.header.frame_id.c_str());
  }

  void goToFirstPose() {
    if (is_task_running_) {
      RCLCPP_WARN(this->get_logger(), "A task is already running. Ignoring first pose.");
      return;
    }

    if (!first_pose_) {
        RCLCPP_ERROR(get_logger(), "First pose not available yet!");
        is_task_running_ = false;
        return;
    }

    is_task_running_ = true;

    RCLCPP_INFO(this->get_logger(), "START navigation to FIRST pose");

    // Creiamo il goal per navigare a questo punto
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose = *first_pose_;

    if (!nav2_client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available. Aborting task.");
      is_task_running_ = false;
      return;
    }

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.result_callback =
      [this](const GoalHandlePose::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "Reached first coverage pose!");
          startCoverage();
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to reach first coverage pose.");

          // Prova comunque a fare startCoverage.  Mal che vada il coverage server risponde di no
          if (numAttempts < 3) {
            numAttempts++;
            startCoverage();
          }
        }
        is_task_running_ = false;
      };

    nav2_client_->async_send_goal(goal_msg, send_goal_options);
  }


  void startCoverage()
  {
    if (!coverage_client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Coverage action server not available. Aborting task.");
      is_task_running_ = false;
      return;
    }

    auto goal_msg = NavigateCoverage::Goal();
    goal_msg.frame_id = "map";
    goal_msg.polygons.push_back(toPolygon(field));

    RCLCPP_INFO(this->get_logger(), "Starting complete coverage task.");

    auto send_goal_options = rclcpp_action::Client<NavigateCoverage>::SendGoalOptions();
    
    // Callback per il risultato finale della copertura
    send_goal_options.result_callback = 
      [this](const GoalHandleCoverage::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "Coverage task completed successfully!");
          this->is_task_running_ = false; 
        } else {
          RCLCPP_ERROR(this->get_logger(), "Coverage task failed or was canceled.");

          // Hack per far partire la navigazione verso la prima posa
          // Il CoverageServer invia la prima posa quando parte un tentativo di coverage.
          // Faccio partire solo quando il coverage fallisce.
          // Fa un bel po' schifo, ma è la soluzione più semplice.
          this->is_task_running_ = false;
          goToFirstPose();
        }
      };

    coverage_client_->async_send_goal(goal_msg, send_goal_options);
  }

  rclcpp_action::Client<NavigateCoverage>::SharedPtr coverage_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr coverage_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr first_pose_sub_;
  bool is_task_running_ = false;
  std::vector<std::array<double, 2>> field;
  geometry_msgs::msg::PoseStamped::SharedPtr first_pose_ = nullptr;
  int numAttempts = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CoverageNavigatorTester>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}