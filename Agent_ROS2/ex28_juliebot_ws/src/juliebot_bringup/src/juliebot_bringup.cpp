/**
 * ROS2
 * 
 * 1. receive MicroROS's odom_msg (/odom)
 * 2. broadcaster TF (coordinate transform info) on ROS
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

class TopicSubscribe01 : public rclcpp::Node
{
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    nav_msgs::msg::Odometry odom_msg_;

    /**
     * user: odom_sub_
     * callback: to deal with the received odom_msg_
     */
    void callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg);

public:
    /**
     * initialize odom_sub_, tf_broadcaster_
     */
    TopicSubscribe01(std::string name);

    /**
     * user: tf_broadcaster_
     * purpose: to broadcaster tf msg
     */
    void publish_tf();
};

TopicSubscribe01::TopicSubscribe01(std::string name) : Node(name)
{
    /**
     * create a sub
     * topic: odom
     * msg type: nav_msgs::msg::Odometry::SharedPtr
     */
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS(), std::bind(&TopicSubscribe01::callback_odom, this, std::placeholders::_1)); // placeholders::_1 ???? define

    /**
     * create a tf_broadercaster_(node)
     */
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
}

void TopicSubscribe01::callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "recv odom_msg: base_footprint_tf: <%f, %f>", msg->pose.pose.position.x, msg->pose.pose.position.y);

    // update odom_msg_ (pose)
    odom_msg_.pose.pose.position.x = msg->pose.pose.position.x;
    odom_msg_.pose.pose.position.y = msg->pose.pose.position.y;
    odom_msg_.pose.pose.position.z = msg->pose.pose.position.z;

    odom_msg_.pose.pose.orientation.w = msg->pose.pose.orientation.w;
    odom_msg_.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    odom_msg_.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    odom_msg_.pose.pose.orientation.z = msg->pose.pose.orientation.z;
}

void TopicSubscribe01::publish_tf()
{
    geometry_msgs::msg::TransformStamped tf;

    double seconds = this->now().seconds();
    tf.header.stamp = rclcpp::Time(static_cast<uint64_t>(seconds * 1e9));
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_footprint";

    tf.transform.translation.x = odom_msg_.pose.pose.position.x;
    tf.transform.translation.y = odom_msg_.pose.pose.position.y;
    tf.transform.translation.z = odom_msg_.pose.pose.position.z;

    tf.transform.rotation.w = odom_msg_.pose.pose.orientation.w;
    tf.transform.rotation.x = odom_msg_.pose.pose.orientation.x;
    tf.transform.rotation.y = odom_msg_.pose.pose.orientation.y;
    tf.transform.rotation.z = odom_msg_.pose.pose.orientation.z;

    // broadcaster tf
    tf_broadcaster_->sendTransform(tf);
}

int main(int argc, char **argv)
{
    // 1. initialize ros2 node
    rclcpp::init(argc, argv);

    // 2. create TopicSubscribe01 node
    auto node = std::make_shared<TopicSubscribe01>("juliebot_bringup");

    // 3. spin node
    rclcpp::WallRate loop_rate(20.0); // loop frequency
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node); // callback

        node->publish_tf(); // broadcaster TF info

        loop_rate.sleep();
    }

    // 4. close node
    rclcpp::shutdown();
    return 0;
}
