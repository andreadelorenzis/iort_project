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

    polygon_sub_ = this->create_subscription<geometry_msgs::msg::Polygon>(
      "/coverage_zone", 10, std::bind(&CoverageNavigatorTester::polygonCallback, this, std::placeholders::_1));

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

  // --- Logica principale ---

  // 1. Callback che riceve il poligono e inizia la sequenza.
  void polygonCallback(const geometry_msgs::msg::Polygon::SharedPtr msg)
  {
    if (is_task_running_) {
      RCLCPP_WARN(this->get_logger(), "A task is already running. Ignoring new polygon.");
      return;
    }

    if (msg->points.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty polygon, ignoring.");
      return;
    }

    std::vector<std::array<double, 2>> field;
    for (const auto &pt : msg->points) {
      field.push_back({pt.x, pt.y});
    }

    RCLCPP_INFO(this->get_logger(), "Received polygon with %zu points. Starting navigation sequence.", field.size());
    is_task_running_ = true;

    // Inizia la prima azione della catena: andare al punto di partenza.
    goToStartPoint(field);
  }

  // 2. Funzione che invia il goal per raggiungere il punto di partenza.
  void goToStartPoint(const std::vector<std::array<double, 2>>& field)
  {
    if (!nav2_client_->wait_for_action_server(10s)) {
        RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available. Aborting task.");
        is_task_running_ = false;
        return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();

    double safe_margin = 0.5;
    double entry_x = field[0][0] + safe_margin;
    double entry_y = field[0][1] + safe_margin;

    goal_msg.pose.pose.position.x = entry_x;
    goal_msg.pose.pose.position.y = entry_y;
    goal_msg.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "Sending goal to reach start point: (%.2f, %.2f)", entry_x, entry_y);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    
    // **LA PARTE CRUCIALE**: La callback del risultato di questa azione
    // innescherà l'azione successiva.
    send_goal_options.result_callback = 
      [this, field](const GoalHandlePose::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "Successfully reached start point!");
          // Se ha avuto successo, avvia la copertura.
          this->startCoverage(field);
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to reach start point. Aborting task.");
          this->is_task_running_ = false;
        }
      };

    nav2_client_->async_send_goal(goal_msg, send_goal_options);
  }

  // 3. Funzione che invia il goal per iniziare la copertura.
  void startCoverage(const std::vector<std::array<double, 2>>& field)
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
        } else {
          RCLCPP_ERROR(this->get_logger(), "Coverage task failed or was canceled.");
        }
        this->is_task_running_ = false; // La task è finita, pronto per la prossima.
      };

    coverage_client_->async_send_goal(goal_msg, send_goal_options);
  }

  rclcpp_action::Client<NavigateCoverage>::SharedPtr coverage_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr polygon_sub_;
  bool is_task_running_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CoverageNavigatorTester>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}