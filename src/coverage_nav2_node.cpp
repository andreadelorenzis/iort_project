#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "action_msgs/msg/goal_status.hpp"

#include "vacuum_bot/msg/coverage_zone.hpp"
#include "vacuum_bot/msg/coverage_plan.hpp"
#include "vacuum_bot/msg/coverage_mission.hpp"

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

    // Subscriber per singolo poligono
    coverage_sub_ = this->create_subscription<geometry_msgs::msg::Polygon>(
      "/coverage_start", 10, std::bind(&CoverageNavigatorTester::coverageCallback, this, std::placeholders::_1));

    // Subscriber per primo pose
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    first_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/coverage_first_pose", qos,
      std::bind(&CoverageNavigatorTester::firstPoseCallback, this, std::placeholders::_1));

    // Subscriber per cleaning plan
    mission_sub_ = this->create_subscription<vacuum_bot::msg::CoverageMission>(
      "/cleaning_mission", 10, std::bind(&CoverageNavigatorTester::missionCallback, this, std::placeholders::_1));

    // Publisher for zone cleaning intensity 
    intensity_pub_ = this->create_publisher<std_msgs::msg::Int32>("/cmd_intensity", 10);

    debug_pub_ = this->create_publisher<std_msgs::msg::String>("/debug", 10);

    RCLCPP_INFO(this->get_logger(), "Coverage node ready.");

    publishDebug("CoverageNavigatorTester initialized");
  }

private:

  // --- Utility functions ---

  void publishDebug(const std::string & msg, const std::string & caller = "CoverageNavigator") {
      std_msgs::msg::String debug_msg;
      debug_msg.data = "[" + caller + "] " + msg;
      debug_pub_->publish(debug_msg);
  }

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
    publishDebug("Received coverage polygon with " + std::to_string(msg->points.size()) + " points");

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

  void missionCallback(const vacuum_bot::msg::CoverageMission::SharedPtr msg)
  {
      if (msg->plan.zones.empty()) {
          RCLCPP_WARN(this->get_logger(), "Received mission with no zones.");
          return;
      }

      publishDebug("Received cleaning mission with " + 
                  std::to_string(msg->plan.zones.size()) + " zones and base pose");

      zone_queue_ = msg->plan.zones;
      base_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(msg->base_pose);

      processNextZone();
  }

  void firstPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    publishDebug("Received first pose x=" + std::to_string(msg->pose.position.x) +
                    " y=" + std::to_string(msg->pose.position.y));

    auto fixed_pose = *msg;
    fixed_pose.header.frame_id = "map";
    first_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(fixed_pose);
    RCLCPP_INFO(this->get_logger(), "Received start pose: x=%.2f, y=%.2f (frame=%s)",
                fixed_pose.pose.position.x, fixed_pose.pose.position.y,
                fixed_pose.header.frame_id.c_str());
  }

  void processNextZone()
  {
      if (zone_queue_.empty()) {
          RCLCPP_INFO(this->get_logger(), "All zones completed. Navigating to base...");
          publishDebug("All zones completed. Navigating to base...");

          if (base_pose_) {
              goToBasePose();
          } else {
              RCLCPP_WARN(this->get_logger(), "No base pose provided, stopping mission.");
          }

          is_task_running_ = false;
          return;
      }

      auto zone = zone_queue_.front();
      publishDebug("Processing zone " + std::to_string(zone.zone_id) +
                  " with " + std::to_string(zone.polygon.points.size()) + " points");

      current_zone_ = zone;
      current_zone_repeats_ = current_zone_.frequency;
      publishDebug("Zone frequency set to value: " + std::to_string(current_zone_repeats_));
      zone_queue_.erase(zone_queue_.begin());

      if (zone.polygon.points.empty()) {
          RCLCPP_WARN(this->get_logger(), "Skipping empty zone polygon");
          processNextZone(); 
          return;
      }


      // Converte la zona in field
      field.clear();
      for (const auto &pt : zone.polygon.points) {
          field.push_back({pt.x, pt.y});
      }

      is_task_running_ = true;
      numAttempts = 0;
      startCoverage(); 
  }

  void sendDefaultIntensity() {
    std_msgs::msg::Int32 intensity_msg;
    intensity_msg.data = 1;
    intensity_pub_->publish(intensity_msg);
    publishDebug("Published intensity " + std::to_string(1));
  }


  void goToFirstPose() {
    publishDebug("Navigating to first pose");

    sendDefaultIntensity();

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
          publishDebug("Reached first coverage pose for zone " + std::to_string(current_zone_.zone_id));

          std_msgs::msg::Int32 intensity_msg;
          intensity_msg.data = current_zone_.intensity;
          intensity_pub_->publish(intensity_msg);
          publishDebug("Published intensity " + std::to_string(current_zone_.intensity) +
              " for zone " + std::to_string(current_zone_.zone_id));

          startCoverage();
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to reach first coverage pose.");
          publishDebug("Reached first coverage pose for zone " + std::to_string(current_zone_.zone_id));

          // Prova comunque a fare startCoverage per un certo numero di tentativi.  
          // Mal che vada il coverage server risponde di no
          if (numAttempts < 3) {
            numAttempts++;
            startCoverage();
          }
        }
        is_task_running_ = false;
      };

    nav2_client_->async_send_goal(goal_msg, send_goal_options);
  }


  void goToBasePose() {
    publishDebug("Navigating to base pose");

    sendDefaultIntensity();

    if (!base_pose_) {
        RCLCPP_ERROR(get_logger(), "Base pose not available!");
        return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = *base_pose_;
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();

    if (!nav2_client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available. Cannot go to base.");
      return;
    }

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.result_callback =
      [this](const GoalHandlePose::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "Reached base successfully!");
          publishDebug("Reached base successfully!");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to reach base.");
          publishDebug("Failed to reach base.");
        }
      };

    nav2_client_->async_send_goal(goal_msg, send_goal_options);
  }


  void startCoverage()
  {
    publishDebug("Starting complete coverage task");

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
          publishDebug("Coverage task completed successfully for zone " + std::to_string(current_zone_.zone_id));

          current_zone_repeats_--;
          publishDebug("current_zone_repeats_: " + std::to_string(current_zone_repeats_));
          if (current_zone_repeats_ > 0) {
              RCLCPP_INFO(this->get_logger(), "Repeating zone %d, remaining repeats: %d", 
                current_zone_.zone_id, current_zone_repeats_);
                publishDebug("Repeating zone " + std::to_string(current_zone_.zone_id) 
                  + ", remaining repeats: " + std::to_string(current_zone_repeats_));
              startCoverage();  // ripete la stessa zona
          } else {
              is_task_running_ = false; 
              publishDebug("Zone " + std::to_string(current_zone_.zone_id) + " finished. Going to next zone");
              processNextZone();  // passa alla prossima zona
          }
        } else {
          RCLCPP_ERROR(this->get_logger(), "Coverage task failed or was canceled.");

          // Hack per far partire la navigazione verso la prima posa
          // Il CoverageServer invia la prima posa quando parte un tentativo di coverage.
          // Se il coverage fallisce, di solito significa che il robot non si trova vicino la posizione di
          // partenza del coverage. Pertanto, quando fallisce, faccio andare il robot alla posa iniziale.
          // Fa un po' schifo, ma funziona.
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
  rclcpp::Subscription<vacuum_bot::msg::CoverageMission>::SharedPtr mission_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr intensity_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  bool is_task_running_ = false;
  std::vector<std::array<double, 2>> field;
  std::vector<vacuum_bot::msg::CoverageZone> zone_queue_;
  geometry_msgs::msg::PoseStamped::SharedPtr first_pose_ = nullptr;
  geometry_msgs::msg::PoseStamped::SharedPtr base_pose_ = nullptr;
  vacuum_bot::msg::CoverageZone current_zone_;
  int current_zone_repeats_ = 0;
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