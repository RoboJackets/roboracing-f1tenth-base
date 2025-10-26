#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace lidar {

class LidarNode : public rclcpp::Node
{
public:
    LidarNode(const rclcpp::NodeOptions &options)
        : Node("lidarnode", options)
    {
        dummy_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // Create TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz
            std::bind(&LidarNode::publish_data, this)
        );

        RCLCPP_INFO(this->get_logger(), "LidarNode started: publishing /odom and TFs");
    }

private:
    void publish_data()
    {
        auto now = this->get_clock()->now();

        // --- 1. Publish /odom ---
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // Pose at origin
        odom_msg.pose.pose.position.x = 0.0;
        odom_msg.pose.pose.position.y = 0.0;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = 0.0;
        odom_msg.pose.pose.orientation.w = 1.0;

        // Zero velocity
        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;

        dummy_pub_->publish(odom_msg);

        // --- 2. Broadcast TF: odom -> base_link ---
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = now;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";
        odom_tf.transform.translation.x = 0.0;
        odom_tf.transform.translation.y = 0.0;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation = odom_msg.pose.pose.orientation;

        tf_broadcaster_->sendTransform(odom_tf);

        // --- 3. Broadcast TF: base_link -> laser (optional) ---
        geometry_msgs::msg::TransformStamped laser_tf;
        laser_tf.header.stamp = now;
        laser_tf.header.frame_id = "base_link";
        laser_tf.child_frame_id = "laser";  // match your LiDAR frame name
        laser_tf.transform.translation.x = 0.0;
        laser_tf.transform.translation.y = 0.0;
        laser_tf.transform.translation.z = 0.0;
        laser_tf.transform.rotation.x = 0.0;
        laser_tf.transform.rotation.y = 0.0;
        laser_tf.transform.rotation.z = 0.0;
        laser_tf.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(laser_tf);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr dummy_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(lidar::LidarNode)

}  // namespace lidar
