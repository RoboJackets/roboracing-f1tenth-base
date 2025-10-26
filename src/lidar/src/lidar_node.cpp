#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace lidar {

class LidarNode : public rclcpp::Node
{
public:
    LidarNode(const rclcpp::NodeOptions &options) : Node("lidarnode", options)
    {
        dummy_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // 10 Hz
            std::bind(&LidarNode::publish_odom, this)
        );
        // std::cout << "Started" << std::endl;
    }
private:
    void publish_odom()
    {
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link";

        // Pose at origin
        msg.pose.pose.position.x = 0.0;
        msg.pose.pose.position.y = 0.0;
        msg.pose.pose.position.z = 0.0;

        // Facing forward (no rotation)
        msg.pose.pose.orientation.x = 0.0;
        msg.pose.pose.orientation.y = 0.0;
        msg.pose.pose.orientation.z = 0.0;
        msg.pose.pose.orientation.w = 1.0;

        // No linear or angular velocity
        msg.twist.twist.linear.x = 0.0;
        msg.twist.twist.linear.y = 0.0;
        msg.twist.twist.linear.z = 0.0;
        msg.twist.twist.angular.x = 0.0;
        msg.twist.twist.angular.y = 0.0;
        msg.twist.twist.angular.z = 0.0;

        // std::cout << "Publishing\n";

        dummy_pub_->publish(msg);

        
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr dummy_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(lidar::LidarNode)

}
