#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class LidarNode : public rclcpp::Node
{
public:
    LidarNode()
        : Node("lidar_node"),
          obstacle_distance_(5.0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/lidar_distance",
            10
        );

        timer_ = this->create_wall_timer(
            1s,
            std::bind(&LidarNode::publishDistance, this)
        );
    }

private:
    void publishDistance()
    {
        std_msgs::msg::Float64 message;

        message.data = obstacle_distance_;

        publisher_->publish(message);

        RCLCPP_INFO(
            this->get_logger(),
            "LiDAR distance: %.2f m",
            obstacle_distance_
        );

        obstacle_distance_ -= 0.5;

        if (obstacle_distance_ < 0.5)
        {
            obstacle_distance_ = 5.0;
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    double obstacle_distance_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<LidarNode>()
    );

    rclcpp::shutdown();

    return 0;
}
