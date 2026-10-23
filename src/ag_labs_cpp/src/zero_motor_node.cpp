#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

// Publishes a constant zero velocity on /cmd_vel/zero. It is wired into the
// twist_mux at the lowest priority so the robot always has a safe "stop"
// fallback whenever no higher-priority source (wasd, path follower) is active.
//
// Publishes plain geometry_msgs/Twist (not TwistStamped) to match the other
// command sources and the disc_robot Gazebo bridge, which all use plain Twist.
class ZeroMotorNode : public rclcpp::Node {
public:
    ZeroMotorNode() : Node("zero_motor_node") {
        // Declare parameters
        this->declare_parameter<int>("interval_ms", 24);
        this->declare_parameter<std::string>("state_topic", "/state_node/current_state");

        const int interval_ms = this->get_parameter("interval_ms").as_int();
        const auto state_topic = this->get_parameter("state_topic").as_string();

        // Create publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel/zero", 10);

        // Subscribe to state topic
        state_sub_ = this->create_subscription<std_msgs::msg::String>(
            state_topic, 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                current_state_ = msg->data;
            });

        // Create timer with the configured interval
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(interval_ms),
            std::bind(&ZeroMotorNode::publish_zero_velocity, this)
        );

        RCLCPP_INFO(this->get_logger(), "Zero Motor Node started with interval: %d ms", interval_ms);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string current_state_{"just_turned_on"};

    void publish_zero_velocity() {
        // Only publish zero when NOT in active driving states
        if (current_state_ == "mapping" || current_state_ == "drive_forward" || current_state_ == "navigation") {
            return;
        }

        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.0;

        publisher_->publish(msg);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ZeroMotorNode>());
    rclcpp::shutdown();
    return 0;
}
