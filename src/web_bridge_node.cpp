#include <rclcpp/rclcpp.hpp>
#include <string>
#include <stdio.h>
#include <memory>
#include <iostream>
#include <chrono>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include "easywsclient.hpp"
#include "nlohmann/json.hpp"
#include "std_msgs/msg/string.hpp"

using json = nlohmann::json;
using std::placeholders::_1;
using easywsclient::WebSocket;
static WebSocket::pointer ws = NULL;

class WebBridge : public rclcpp::Node
{
public:
    WebBridge() : Node("web_bridge")
    {
        this->declare_parameter<std::string>("ip", "localhost");
        this->declare_parameter<int>("port", 8080);
        auto ip = this->get_parameter("ip").as_string();
        auto port = this->get_parameter("port").as_int();

        RCLCPP_INFO(this->get_logger(), "Connecting to web socket server at %s:%ld",
            ip.c_str(), port);

        std::stringstream url;
        url << "ws://" << ip << ":" << port;

        while(!rclcpp::ok() || !ws) {
            ws = WebSocket::from_url(url.str());
            if (!ws) {
                RCLCPP_WARN(this->get_logger(), "Failed to connect to web socket server at %s:%ld, retrying...",
                            ip.c_str(), port);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }

        ws->send("Hello!");
        ws->poll();
        RCLCPP_INFO(this->get_logger(), "Connected to web socket server at %s:%ld", 
                    ip.c_str(), port);

        pub_ = this->create_publisher<geometry_msgs::msg::Polygon>(
            "/coverage_zone", 
            10);
        RCLCPP_INFO(this->get_logger(), "Publisher on /coverage_zone created");

        coverage_pub_ = this->create_publisher<geometry_msgs::msg::Polygon>(
            "/coverage_start", 
            10);
        RCLCPP_INFO(this->get_logger(), "Publisher on /coverage_start created");

        manual_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/manual_cmd", 
            10);
        RCLCPP_INFO(this->get_logger(), "Publisher on /manual_cmd created");

        // Timer to poll websocket every 50ms
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&WebBridge::pollWebSocket, this)
        );
    }

    ~WebBridge()
    {
        if (sock_ >= 0) {
            close(sock_);
        }
    }

private:

    void pollWebSocket()
    {
        if (!ws) return;

        ws->poll();
        ws->dispatch([this](const std::string & message) {
            RCLCPP_INFO(this->get_logger(), "Received message: %s", message.c_str());

            try {
                // Parse JSON
                auto j = json::parse(message);

                // Polygon normale
                if (j.contains("type") && j["type"] == "polygon" && j.contains("points")) {
                    geometry_msgs::msg::Polygon poly;

                    for (auto &pt : j["points"]) {
                        geometry_msgs::msg::Point32 p;
                        // Convert string to float
                        p.x = std::stof(pt["x"].get<std::string>());
                        p.y = std::stof(pt["y"].get<std::string>());
                        p.z = 0.0;
                        poly.points.push_back(p);
                    }

                    if (!poly.points.empty()) {
                        pub_->publish(poly);
                        RCLCPP_INFO(this->get_logger(), "Published polygon with %zu points", poly.points.size());
                    }
                }

                // coverage
                else if (j.contains("type") && j["type"] == "coverage" && j.contains("points")) {
                    geometry_msgs::msg::Polygon poly;
                    for (auto &pt : j["points"]) {
                        geometry_msgs::msg::Point32 p;
                        p.x = std::stof(pt["x"].get<std::string>());
                        p.y = std::stof(pt["y"].get<std::string>());
                        p.z = 0.0;
                        poly.points.push_back(p);
                    }
                    if (!poly.points.empty()) {
                        coverage_pub_->publish(poly);
                        RCLCPP_INFO(this->get_logger(), "Published coverage polygon with %zu points", poly.points.size());
                    }
                }

                // manual
                else if (j.contains("type") && j["type"] == "manual" && j.contains("command")) {
                    std::string command = j["command"].get<std::string>();
                    std_msgs::msg::String msg;
                    msg.data = command;
                    manual_pub_->publish(msg);
                    RCLCPP_INFO(this->get_logger(), "Published command: %s", command.c_str());
                }

            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", e.what());
            }
        });
    }

    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr pub_;
    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr coverage_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr manual_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int sock_{-1};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}