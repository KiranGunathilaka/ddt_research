#include <rclcpp/rclcpp.hpp>
#include <eufs_msgs/msg/car_state.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

class OdometryToTwistConverter : public rclcpp::Node {
public:
    OdometryToTwistConverter() : Node("odometry_to_twist_converter") {
        // Subscribe to odometry integration topic
        odometry_sub_ = this->create_subscription<eufs_msgs::msg::CarState>(
            "/odometry_integration/car_state", 10,
            std::bind(&OdometryToTwistConverter::odometryCallback, this, std::placeholders::_1)
        );

        // Publish to ros_can/twist topic
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/ros_can/twist", 10
        );

        RCLCPP_INFO(this->get_logger(), "Odometry to Twist converter initialized");
        RCLCPP_INFO(this->get_logger(), "Converting /odometry_integration/car_state -> /ros_can/twist");
    }

private:
    void odometryCallback(const eufs_msgs::msg::CarState::SharedPtr msg) {
        auto twist_msg = geometry_msgs::msg::TwistWithCovarianceStamped();
        
        // Copy header information
        twist_msg.header.stamp = msg->header.stamp;
        twist_msg.header.frame_id = "base_footprint";  // Standard frame for twist messages
        
        // Copy twist data directly from CarState
        twist_msg.twist.twist.linear.x = msg->twist.twist.linear.x;
        twist_msg.twist.twist.linear.y = msg->twist.twist.linear.y;
        twist_msg.twist.twist.linear.z = msg->twist.twist.linear.z;
        
        twist_msg.twist.twist.angular.x = msg->twist.twist.angular.x;
        twist_msg.twist.twist.angular.y = msg->twist.twist.angular.y;
        twist_msg.twist.twist.angular.z = msg->twist.twist.angular.z;
        
        // Copy covariance matrix
        twist_msg.twist.covariance = msg->twist.covariance;
        
        // Publish the converted message
        twist_pub_->publish(twist_msg);
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Converted and published twist - linear.x: %f, angular.z: %f", 
            twist_msg.twist.twist.linear.x, 
            twist_msg.twist.twist.angular.z);
    }

    // Subscriber
    rclcpp::Subscription<eufs_msgs::msg::CarState>::SharedPtr odometry_sub_;
    
    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryToTwistConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 