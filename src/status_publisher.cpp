#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_custom_msgs/msg/robot_status.hpp"

using namespace std::chrono_literals;

class StatusPublisher : public rclcpp::Node
{
public:
    StatusPublisher()
        : Node("status_publisher"), battery_level_(100.0), mission_count_(0)
    {
        publisher_ = this->create_publisher<ros2_custom_msgs::msg::RobotStatus>(
            "/robot_status", 10);

        // Optimization: Using a lambda function instead of std::bind is the 
        // preferred modern C++ approach. It is often faster and easier to read.
        timer_ = this->create_wall_timer(
            1000ms, [this]() { timer_callback(); });
    }

private:
    void timer_callback()
    {
        // Direct initialization is slightly cleaner than 'auto message = ...()'
        ros2_custom_msgs::msg::RobotStatus message;
        
        message.robot_name = "Explorer1";
        message.battery_level = battery_level_;
        message.is_active = true;
        message.mission_count = mission_count_;

        RCLCPP_INFO(this->get_logger(), "Publishing: robot=%s, battery=%.1f, active=%s, missions=%d",
                    message.robot_name.c_str(),
                    message.battery_level,
                    message.is_active ? "true" : "false",
                    message.mission_count);

        publisher_->publish(message);

        battery_level_ -= 0.5;
        mission_count_ += 1;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ros2_custom_msgs::msg::RobotStatus>::SharedPtr publisher_;
    
    double battery_level_;
    int32_t mission_count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatusPublisher>());
    rclcpp::shutdown();
    return 0;
}
