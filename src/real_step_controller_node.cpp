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
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "easywsclient.hpp"
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <fstream>
#include <map>
#include "std_msgs/msg/string.hpp"
#include <pigpiod_if2.h>

struct IRSignal {
    std::string type; // "pulse" o "space"
    int duration;     // microsecondi
};

const int IR_RX_PIN = 27;   // GPIO ricevitore IR
const int IR_TX_PIN = 22;   // GPIO LED IR
const int CARRIER_FREQ = 38000; // Hz (38 kHz standard)
const double DUTY_CYCLE = 0.33; // 33%
const int DELAY_BEFORE_SEND = 5; // secondi

enum class RobotState {
    STOP,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};
using std::placeholders::_1;

class StepController : public rclcpp::Node
{
public:
    StepController(const rclcpp::NodeOptions & options)
    : Node("step_controller", options)
    {
        this->declare_parameter<std::string>("signals_dir", "");
        signals_dir_path_ = this->get_parameter("signals_dir").as_string();
        RCLCPP_INFO(this->get_logger(), "Signals dir path: %s", signals_dir_path_.c_str());

        ir_signals[RobotState::FORWARD]  = read_ir_from_file(signals_dir_path_ + "/ir_forward.txt");
        ir_signals[RobotState::BACKWARD] = read_ir_from_file(signals_dir_path_ + "/ir_backward.txt");
        ir_signals[RobotState::LEFT]     = read_ir_from_file(signals_dir_path_ + "/ir_left.txt");
        ir_signals[RobotState::RIGHT]    = read_ir_from_file(signals_dir_path_ + "/ir_right.txt");

        pi = pigpio_start(NULL, NULL);
        if (pi < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error during connection to pigpiod");
        } else {
            RCLCPP_INFO(this->get_logger(), "Connected to pigpio deamon!");
        }

        subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel_nav", rclcpp::QoS(10).reliable(),
        std::bind(&StepController::nav_cmd_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Subscription on /cmd_vel created!");

        manual_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/manual_cmd", 10, std::bind(&StepController::manual_cmd_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Subscription on /manual_cmd created!");

        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&StepController::send_command, this));

        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&StepController::check_watchdog, this));

        last_cmd_time_ = now();
        current_state = RobotState::STOP;
    }

    ~StepController() {
        pigpio_stop(pi);
    }

    private:

    void nav_cmd_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        last_cmd_time_ = now();
        const double EPS = 1e-3;
        RobotState new_state = RobotState::STOP;

        double abs_linear  = std::abs(msg->twist.linear.x);
        double abs_angular = std::abs(msg->twist.angular.z);

        if (std::abs(msg->twist.linear.x) > EPS &&  abs_linear > abs_angular) {
            new_state = (msg->twist.linear.x > 0) ? RobotState::FORWARD : RobotState::BACKWARD;
        } else if (std::abs(msg->twist.angular.z) > EPS && abs_linear <= abs_angular) {
            new_state = (msg->twist.angular.z > 0) ? RobotState::LEFT : RobotState::RIGHT;
        }

        if (new_state != current_state) {
            current_state = new_state;
            RCLCPP_INFO(this->get_logger(), "Current state: %s", state_to_string(current_state).c_str());
        }
    }

    void manual_cmd_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string cmd = msg->data;
        if (cmd == "LEFT")       current_state = RobotState::LEFT;
        else if (cmd == "RIGHT") current_state = RobotState::RIGHT;
        else if (cmd == "FORWARD") current_state = RobotState::FORWARD;
        else if (cmd == "STOP")  current_state = RobotState::STOP;
    }


    void send_command()
    {
        std::vector<IRSignal> signal;
        switch(current_state)
        {
        case RobotState::FORWARD:
            signal = ir_signals[RobotState::FORWARD];
            break;
        case RobotState::LEFT:
            signal = ir_signals[RobotState::LEFT];
            break;
        case RobotState::RIGHT:
            signal = ir_signals[RobotState::RIGHT];
            break;
        case RobotState::STOP:
        default:
            break;
        }
        send_raw_wave(pi, signal, IR_TX_PIN, CARRIER_FREQ, DUTY_CYCLE);
    }

    void check_watchdog()
    {
        auto now_time = now();
        auto dt = (now_time - last_cmd_time_).seconds();
        if (dt > 0.2 && current_state != RobotState::STOP) { // 200ms timeout
            current_state = RobotState::STOP;
            RCLCPP_WARN(this->get_logger(), "Watchdog triggered: stopping robot");
        }
    }

    std::vector<IRSignal> read_ir_from_file(const std::string& filename) {
        std::vector<IRSignal> signal;
        std::ifstream infile(filename);
        if (!infile.is_open()) {
            std::cerr << "Errore: impossibile aprire file IR: " << filename << std::endl;
            return signal;
        }

        std::string type;
        int duration;
        while (infile >> type >> duration) {
            signal.push_back({type, duration});
        }
        return signal;
    }


    // --- FUNZIONE PER TRASMETTERE ---
    void send_raw_wave(int pi, const std::vector<IRSignal>& signal,
                    int gpio_pin, int carrier_freq, double duty_cycle) {
        int micros_per_cycle = static_cast<int>(1'000'000 / carrier_freq);
        int on_micros = static_cast<int>(micros_per_cycle * duty_cycle);
        int off_micros = micros_per_cycle - on_micros;

        std::vector<gpioPulse_t> wf;

        for (const auto& s : signal) {
            if (s.type == "pulse") {
                int cycles = s.duration / micros_per_cycle;
                for (int i = 0; i < cycles; i++) {
                    gpioPulse_t p1 = {1u << gpio_pin, 0, static_cast<uint32_t>(on_micros)};
                    gpioPulse_t p2 = {0, 1u << gpio_pin, static_cast<uint32_t>(off_micros)};
                    wf.push_back(p1);
                    wf.push_back(p2);
                }
            } else if (s.type == "space") {
                gpioPulse_t space = {0, 0, static_cast<uint32_t>(s.duration)};
                wf.push_back(space);
            }
        }

        wave_clear(pi);
        wave_add_generic(pi, wf.size(), wf.data());
        int wave_id = wave_create(pi);

        if (wave_id >= 0) {
            wave_send_once(pi, wave_id);
            while (wave_tx_busy(pi)) {
                time_sleep(0.001); // attesa fine trasmissione
            }
            wave_delete(pi, wave_id);
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


    const std::map<RobotState, std::pair<double,double>> command_map = {
        {RobotState::FORWARD,  {0.9,  0.0}},
        {RobotState::BACKWARD, {-0.9, 0.0}},
        {RobotState::LEFT,     {0.0,  1.5}},
        {RobotState::RIGHT,    {0.0, -1.5}},
        {RobotState::STOP,     {0.0,  0.0}}
    };

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr manual_cmd_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    rclcpp::Clock::SharedPtr clock_;
    RobotState current_state;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Time last_cmd_time_;

    std::string signals_dir_path_;
    std::map<RobotState, std::vector<IRSignal>> ir_signals;
    int pi;
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

