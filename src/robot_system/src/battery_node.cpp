#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode()
        : Node("battery_node"),
          battery_level_(100.0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/battery",
            10
        );

        timer_ = this->create_wall_timer(
            1s,
            std::bind(&BatteryNode::publishBattery, this)
        );
    }

private:
    void publishBattery()
    {
        std_msgs::msg::Float64 message;

        message.data = battery_level_;

        publisher_->publish(message);

        RCLCPP_INFO(
            this->get_logger(),
            "Battery: %.2f%%",
            battery_level_
        );

        battery_level_ -= 1.0;

        if (battery_level_ < 0.0)
        {
            battery_level_ = 100.0;
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    double battery_level_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<BatteryNode>()
    );

    rclcpp::shutdown();

    return 0;
}
