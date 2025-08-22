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
        ws = WebSocket::from_url(url.str());
        if (!ws) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to web socket server at %s:%ld",
                        ip.c_str(), port);
            throw std::runtime_error("Web socket connection failed");
        }

        ws->send("Hello!");
        ws->poll();
        RCLCPP_INFO(this->get_logger(), "Connected to web socket server at %s:%ld", 
                    ip.c_str(), port);

        pub_ = this->create_publisher<geometry_msgs::msg::Polygon>(
            "/coverage_zone", 
            10);
        RCLCPP_INFO(this->get_logger(), "Publisher on /coverage_zone created");

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
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", e.what());
            }
        });
    }

    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr pub_;
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