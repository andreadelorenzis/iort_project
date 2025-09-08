#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <fstream>

class DebugNode : public rclcpp::Node
{
public:
    DebugNode()
    : Node("debug_node")
    {
        // Apri il file log in append mode
        logfile_.open("debug_log.txt", std::ios::app);

        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/debug",
            10,
            std::bind(&DebugNode::debugCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Debug node started. Listening on /debug");
    }

    ~DebugNode()
    {
        if (logfile_.is_open()) {
            logfile_.close();
        }
    }

private:
    void debugCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        // Ottieni ora corrente
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);

        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_time), "%Y-%m-%d %H:%M:%S");

        // Messaggio formattato
        std::string log_msg = "[" + ss.str() + "] " + msg->data;

        // Stampa in console
        RCLCPP_INFO(this->get_logger(), "%s", log_msg.c_str());

        // Scrivi su file
        if (logfile_.is_open()) {
            logfile_ << log_msg << std::endl;
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::ofstream logfile_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DebugNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
