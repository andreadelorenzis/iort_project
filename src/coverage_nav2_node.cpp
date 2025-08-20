#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

#include <vector>

using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

class SquarePathNode : public rclcpp::Node
{
public:
    SquarePathNode()
    : Node("square_path_node")
    {
        client_ptr_ = rclcpp_action::create_client<FollowWaypoints>(this, "/follow_waypoints");

        generate_square_path();
        send_goal();
    }

private:
    rclcpp_action::Client<FollowWaypoints>::SharedPtr client_ptr_;
    std::vector<geometry_msgs::msg::PoseStamped> square_path_;

    void generate_square_path()
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.z = 0.0;

        std::vector<std::pair<double,double>> points = {
            {0.0, 0.0},  // start
            {1.0, 0.0},
            {1.0, 1.0},
            {0.0, 1.0},
            {0.0, 0.0}   // close loop
        };

        for (size_t i = 0; i < points.size(); ++i) {
            pose.pose.position.x = points[i].first;
            pose.pose.position.y = points[i].second;

            // compute yaw towards the next point (if not last)
            double yaw = 0.0;
            if (i < points.size() - 1) {
                double dx = points[i+1].first - points[i].first;
                double dy = points[i+1].second - points[i].second;
                yaw = std::atan2(dy, dx);
            }

            // convert yaw to quaternion
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();

            square_path_.push_back(pose);
        }
    }

    void send_goal()
    {
        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "FollowWaypoints action server not available!");
            return;
        }

        auto goal_msg = FollowWaypoints::Goal();
        goal_msg.poses = square_path_;

        auto options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        options.goal_response_callback =
            [](GoalHandleFollowWaypoints::SharedPtr goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was rejected by server");
                } else {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal accepted by server");
                }
            };

        options.feedback_callback =
            [](GoalHandleFollowWaypoints::SharedPtr,
               const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                            "Reached waypoint index: %u", feedback->current_waypoint);
            };

        options.result_callback =
            [](const GoalHandleFollowWaypoints::WrappedResult & result) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                            "Completed square with status: %d", static_cast<int>(result.code));
            };

        client_ptr_->async_send_goal(goal_msg, options);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquarePathNode>());
    rclcpp::shutdown();
    return 0;
}
