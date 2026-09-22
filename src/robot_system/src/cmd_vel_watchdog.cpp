#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class CmdVelWatchdog : public rclcpp::Node
{
public:

    CmdVelWatchdog()
        : Node("cmd_vel_watchdog"),
          last_command_time_(this->now())
    {
        cmd_subscription_ =
            this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel_nav",
                10,
                std::bind(
                    &CmdVelWatchdog::cmdVelCallback,
                    this,
                    std::placeholders::_1
                )
            );

        safe_cmd_publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>(
                "/cmd_vel_safe",
                10
            );

        watchdog_timer_ =
            this->create_wall_timer(
                50ms,
                std::bind(
                    &CmdVelWatchdog::watchdogLoop,
                    this
                )
            );

        RCLCPP_INFO(
            this->get_logger(),
            "CMD_VEL watchdog started. Timeout = 0.5 seconds."
        );
    }

private:

    void cmdVelCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_command_ = *msg;
        last_command_time_ = this->now();
    }

    void watchdogLoop()
    {
        geometry_msgs::msg::Twist safe_command;

        double command_age =
            (this->now() - last_command_time_).seconds();

        if (command_age <= 0.5)
        {
            safe_command = last_command_;
        }
        else
        {
            // No command received recently -> STOP
            safe_command.linear.x = 0.0;
            safe_command.linear.y = 0.0;
            safe_command.linear.z = 0.0;

            safe_command.angular.x = 0.0;
            safe_command.angular.y = 0.0;
            safe_command.angular.z = 0.0;
        }

        safe_cmd_publisher_->publish(safe_command);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        cmd_subscription_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
        safe_cmd_publisher_;

    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    geometry_msgs::msg::Twist last_command_;

    rclcpp::Time last_command_time_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<CmdVelWatchdog>()
    );

    rclcpp::shutdown();

    return 0;
}
