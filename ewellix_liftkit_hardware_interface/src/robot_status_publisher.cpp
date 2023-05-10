#include "../include/liftkit_hardware_interface/robot_status_publisher.hpp"
#include "liftkit_hardware_interface/liftkit_hardware_interface.hpp"


using namespace liftkit_hardware_interface;

RobotStatusPublisher::RobotStatusPublisher()
    : Node("robot_status_publisher"), count_(0)
{
    publisher_ = this->create_publisher<std_msgs::msg::Bool>("io_and_status_controller/robot_program_running", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&RobotStatusPublisher::timer_callback, this));
}

void RobotStatusPublisher::timer_callback()
{
    auto message = std_msgs::msg::Bool();
    message.data = true;
    publisher_->publish(message);
}

// Node execution starts here
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotStatusPublisher>());
    rclcpp::shutdown();
    return 0;
}
