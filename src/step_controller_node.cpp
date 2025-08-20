#include <chrono>
#include <memory>
#include <string>
#include <assert.h>
#include <stdio.h>
#include <mutex>
#include <deque>
#include <thread>
#include <chrono>
#include <atomic>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "easywsclient.hpp"

#include <arpa/inet.h>
#include <unistd.h>

using std::placeholders::_1;
using easywsclient::WebSocket;
static WebSocket::pointer ws = NULL;

enum class RobotState {
    STOP,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

class StepController : public rclcpp::Node
{
public:
    StepController(const rclcpp::NodeOptions & options)
    : Node("step_controller", options)
    {
        // Parameters for ESP32 connection
        this->declare_parameter<bool>("simulation", true);
        simulation_ = this->get_parameter("simulation").as_bool();

        if (!simulation_) {
            clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

            this->declare_parameter<std::string>("esp32_ip", "192.168.1.13");
            this->declare_parameter<int>("esp32_port", 80);
            auto ip = this->get_parameter("esp32_ip").as_string();
            auto port = this->get_parameter("esp32_port").as_int();

            RCLCPP_INFO(this->get_logger(), "Connecting to ESP32 at %s:%ld", ip.c_str(), port);

            std::stringstream url;
            url << "ws://" << ip << ":" << port;
            ws = WebSocket::from_url(url.str());
            if (!ws) {
                RCLCPP_ERROR(this->get_logger(), "Failed to connect to ESP32 at %s:%ld",
                            ip.c_str(), port);
                throw std::runtime_error("ESP32 connection failed");
            }

            ws->send("Hello!");
            ws->poll();
            RCLCPP_INFO(this->get_logger(), "Connected to ESP32 at %s:%ld", 
                        ip.c_str(), port);
        } else {
            clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

            RCLCPP_INFO(this->get_logger(), "Running in simulation mode");
            pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
                "/diff_cont/cmd_vel_unstamped", 
                rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
            RCLCPP_INFO(this->get_logger(), "Publisher on /diff_cont/cmd_vel created");
        }

        lastTime_ = clock_->now();

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::QoS(10).reliable(),
        std::bind(&StepController::cmd_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Subscription on /cmd_vel created!");

        publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(publish_rate), std::bind(&StepController::send_command, this));

        // In constructor
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() {
                if ((clock_->now() - lastTime_).seconds() > 0.5) {
                    if (current_state != RobotState::STOP) {
                        current_state = RobotState::STOP;
                        RCLCPP_WARN(this->get_logger(), "Watchdog triggered: STOP");
                    }
                }
            }
        );

    }

    ~StepController()
    {
        if (sock_ >= 0) {
            close(sock_);
        }
    }

private:

    void send_command()
    {
        if (!this->simulation_) {
            if (ws) {
                ws->send(state_to_string(current_state));
                ws->poll();
            }
        } else {
            auto it = command_map.find(this->current_state);
            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x  = it->second.first;
            msg.angular.z = it->second.second;
            pub_->publish(msg);
        }
    }


    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        rclcpp::Time now = clock_->now();
        rclcpp::Duration dt = (lastTime_.nanoseconds() > 0) 
            ? now - lastTime_ : rclcpp::Duration(0, 0);
        this->lastTime_ = now;

        // Cap dt at:
        // 1 for commands sents manually (very low frequency)
        // 0.01 for very fast frequencies (>20hz)
        double dt_capped = std::clamp(dt.seconds(), 0.01, 1.0);

        acc_linear_x += msg->linear.x * dt_capped;
        acc_angular_z += msg->angular.z * dt_capped;

        RobotState new_state = current_state;
        const double EPS = 1e-3;

        if (std::abs(acc_angular_z) >= ANGULAR_Z_THREESHOLD) {
            new_state = (acc_angular_z > 0.0) ? RobotState::LEFT : RobotState::RIGHT;
            acc_angular_z = 0.0;
        } else if (std::abs(acc_linear_x) >= LINEAR_X_THREESHOLD) {
            new_state = (acc_linear_x > 0.0) ? RobotState::FORWARD : RobotState::BACKWARD;
            acc_linear_x = 0.0;
        }else if (std::abs(msg->angular.z) < EPS && std::abs(msg->linear.x) < EPS) {
            new_state = RobotState::STOP;
        }

        if (new_state != current_state) {
            current_state = new_state;
            RCLCPP_INFO(this->get_logger(), "Current state: %s", state_to_string(current_state).c_str());
        }
    }

    std::string state_to_string(RobotState state) {
        switch (state) {
            case RobotState::STOP:     return "STOP";
            case RobotState::FORWARD:  return "FORWARD";
            case RobotState::BACKWARD: return "BACKWARD";
            case RobotState::LEFT:     return "LEFT";
            case RobotState::RIGHT:    return "RIGHT";
        }
        return "UNKNOWN";
    }

    const int publish_rate = 300; // ms

    const float ANGULAR_Z_THREESHOLD = 0.8; // 30°/s
    const float LINEAR_X_THREESHOLD = 0.4; // 0.3m/s
    const std::map<RobotState, std::pair<double,double>> command_map = {
        {RobotState::FORWARD,  {0.4,  0.0}},
        {RobotState::BACKWARD, {-0.4, 0.0}},
        {RobotState::LEFT,     {0.0,  0.8}},
        {RobotState::RIGHT,    {0.0, -0.8}},
        {RobotState::STOP,     {0.0,  0.0}}
    };
    float acc_linear_x = 0;
    float acc_angular_z = 0;
    int sock_{-1};
    bool simulation_;
    rclcpp::Clock::SharedPtr clock_;


    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Time lastTime_{0, 0, RCL_ROS_TIME};

    std::deque<std::string> command_queue_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    RobotState current_state = RobotState::STOP;
};
  
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("use_sim_time", true)
    });

    auto node = std::make_shared<StepController>(options);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

