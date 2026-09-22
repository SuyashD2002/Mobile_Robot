#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "Robot.h"
#include "StateManager.h"


class RobotControllerNode : public rclcpp::Node
{
public:

    RobotControllerNode()
        : Node("robot_controller_node"),
          robot_("WarehouseBot"),
          state_manager_(0.5, 20.0, 1.0, 2.0),
          lidar_distance_(5.0),
          battery_level_(100.0)
    {
        robot_.setTargetPosition(10.0);


        // =========================================================
        // LiDAR subscription
        // =========================================================

        lidar_subscription_ =
            this->create_subscription<std_msgs::msg::Float64>(
                "/lidar_distance",
                10,
                std::bind(
                    &RobotControllerNode::lidarCallback,
                    this,
                    std::placeholders::_1
                )
            );


        // =========================================================
        // Battery subscription
        // =========================================================

        battery_subscription_ =
            this->create_subscription<std_msgs::msg::Float64>(
                "/battery",
                10,
                std::bind(
                    &RobotControllerNode::batteryCallback,
                    this,
                    std::placeholders::_1
                )
            );


        // =========================================================
        // Velocity publisher
        // =========================================================

        velocity_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/robot_velocity",
                10
            );


        // =========================================================
        // State publisher
        // =========================================================

        state_publisher_ =
            this->create_publisher<std_msgs::msg::String>(
                "/robot_state",
                10
            );


        // =========================================================
        // Command velocity publisher
        // =========================================================

        cmd_vel_publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10
            );


        // =========================================================
        // Emergency reset service
        // =========================================================

        emergency_reset_service_ =
            this->create_service<std_srvs::srv::Trigger>(
                "/emergency_reset",
                std::bind(
                    &RobotControllerNode::emergencyResetCallback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2
                )
            );


        // =========================================================
        // 10 Hz control loop
        // =========================================================

        control_timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(
                    &RobotControllerNode::controlLoop,
                    this
                )
            );


        RCLCPP_INFO(
            this->get_logger(),
            "Robot Controller started."
        );
    }


private:


    // =============================================================
    // Emergency reset callback
    // =============================================================

    void emergencyResetCallback(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        (void)request;

        if (robot_.resetEmergency())
        {
            response->success = true;
            response->message =
                "Emergency reset successful.";

            RCLCPP_WARN(
                this->get_logger(),
                "Emergency reset requested."
            );
        }
        else
        {
            response->success = false;
            response->message =
                "Robot is not in EMERGENCY state.";

            RCLCPP_INFO(
                this->get_logger(),
                "Emergency reset ignored: robot is not in EMERGENCY."
            );
        }
    }


    // =============================================================
    // LiDAR callback
    // =============================================================

    void lidarCallback(
        const std_msgs::msg::Float64::SharedPtr msg)
    {
        lidar_distance_ = msg->data;
    }


    // =============================================================
    // Battery callback
    // =============================================================

    void batteryCallback(
        const std_msgs::msg::Float64::SharedPtr msg)
    {
        battery_level_ = msg->data;
    }


    // =============================================================
    // Main control loop
    // =============================================================

    void controlLoop()
    {
        // ---------------------------------------------------------
        // Evaluate robot safety state
        // ---------------------------------------------------------

        state_manager_.evaluate(
            robot_,
            lidar_distance_,
            battery_level_
        );


        // ---------------------------------------------------------
        // Publish robot velocity
        // ---------------------------------------------------------

        std_msgs::msg::Float64 velocity_message;

        velocity_message.data =
            robot_.getVelocity();

        velocity_publisher_->publish(
            velocity_message
        );


        // ---------------------------------------------------------
        // Publish robot state
        // ---------------------------------------------------------

        std_msgs::msg::String state_message;

        state_message.data =
            robot_.getStateName();

        state_publisher_->publish(
            state_message
        );


        // ---------------------------------------------------------
        // Publish /cmd_vel
        // ---------------------------------------------------------

        geometry_msgs::msg::Twist cmd_vel_message;

        cmd_vel_message.linear.x =
            robot_.getVelocity();

        cmd_vel_message.linear.y = 0.0;
        cmd_vel_message.linear.z = 0.0;

        cmd_vel_message.angular.x = 0.0;
        cmd_vel_message.angular.y = 0.0;
        cmd_vel_message.angular.z = 0.0;

        cmd_vel_publisher_->publish(
            cmd_vel_message
        );


        // ---------------------------------------------------------
        // Console information
        // ---------------------------------------------------------

        RCLCPP_INFO(
            this->get_logger(),
            "LiDAR: %.2f m | Battery: %.2f%% | "
            "Velocity: %.2f m/s | State: %s",
            lidar_distance_,
            battery_level_,
            robot_.getVelocity(),
            robot_.getStateName().c_str()
        );
    }


    // =============================================================
    // Robot objects
    // =============================================================

    Robot robot_;

    StateManager state_manager_;


    // =============================================================
    // Sensor data
    // =============================================================

    double lidar_distance_;

    double battery_level_;


    // =============================================================
    // Subscribers
    // =============================================================

    rclcpp::Subscription<
        std_msgs::msg::Float64
    >::SharedPtr lidar_subscription_;


    rclcpp::Subscription<
        std_msgs::msg::Float64
    >::SharedPtr battery_subscription_;


    // =============================================================
    // Timer
    // =============================================================

    rclcpp::TimerBase::SharedPtr control_timer_;


    // =============================================================
    // Service
    // =============================================================

    rclcpp::Service<
        std_srvs::srv::Trigger
    >::SharedPtr emergency_reset_service_;


    // =============================================================
    // Publishers
    // =============================================================

    rclcpp::Publisher<
        std_msgs::msg::Float64
    >::SharedPtr velocity_publisher_;


    rclcpp::Publisher<
        geometry_msgs::msg::Twist
    >::SharedPtr cmd_vel_publisher_;


    rclcpp::Publisher<
        std_msgs::msg::String
    >::SharedPtr state_publisher_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<RobotControllerNode>()
    );

    rclcpp::shutdown();

    return 0;
}
