#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "your_package_name/msg/robot_status.hpp" 

using namespace std::chrono_literals;

class StatusPublisher : public rclcpp::Node {
public:
    StatusPublisher() 
    : Node("status_publisher"), battery_level_(100.0), mission_count_(0) {
        
        publisher_ = this->create_publisher<your_package_name::msg::RobotStatus>("/robot_status", 10);
        
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&StatusPublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        auto message = your_package_name::msg::RobotStatus();
        
        message.robot_name = "Explorer1";
        message.battery_level = battery_level_;
        message.is_active = true;
        message.mission_count = mission_count_;

        RCLCPP_INFO(this->get_logger(), "Publishing - Name: %s, Battery: %.1f, Missions: %d", 
                    message.robot_name.c_str(), message.battery_level, message.mission_count);

        publisher_->publish(message);

        battery_level_ -= 0.5;
        mission_count_++;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<your_package_name::msg::RobotStatus>::SharedPtr publisher_;
    
    double battery_level_;
    int32_t mission_count_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatusPublisher>());
    rclcpp::shutdown();
    return 0;
}
