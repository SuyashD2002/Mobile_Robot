#include <cmath>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"

class GazeboLidarNode : public rclcpp::Node
{
public:

    GazeboLidarNode()
        : Node("gazebo_lidar_node")
    {
        scan_subscription_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                10,
                std::bind(
                    &GazeboLidarNode::scanCallback,
                    this,
                    std::placeholders::_1
                )
            );

        distance_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/lidar_distance",
                10
            );

        RCLCPP_INFO(
            this->get_logger(),
            "Gazebo LiDAR processor started."
        );
    }

private:

    void scanCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        double minimum_distance =
            std::numeric_limits<double>::infinity();

        /*
         * Check only the front sector:
         *
         * -30 degrees to +30 degrees
         */

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            double angle =
                msg->angle_min +
                i * msg->angle_increment;

            // Front sector: -30° to +30°
            if (angle < -0.523599 || angle > 0.523599)
            {
                continue;
            }

            double range = msg->ranges[i];

            // Ignore invalid measurements
            if (!std::isfinite(range))
            {
                continue;
            }

            if (range < msg->range_min ||
                range > msg->range_max)
            {
                continue;
            }

            if (range < minimum_distance)
            {
                minimum_distance = range;
            }
        }

        std_msgs::msg::Float64 distance_message;

        if (std::isfinite(minimum_distance))
        {
            distance_message.data = minimum_distance;
        }
        else
        {
            // No obstacle in front within LiDAR range
            distance_message.data = 10.0;
        }

        distance_publisher_->publish(distance_message);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Front obstacle distance: %.2f m",
            distance_message.data
        );
    }

    rclcpp::Subscription<
        sensor_msgs::msg::LaserScan
    >::SharedPtr scan_subscription_;

    rclcpp::Publisher<
        std_msgs::msg::Float64
    >::SharedPtr distance_publisher_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<GazeboLidarNode>()
    );

    rclcpp::shutdown();

    return 0;
}
