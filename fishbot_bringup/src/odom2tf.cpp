#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdomTopic2TF : public rclcpp::Node {
public:
  OdomTopic2TF(std::string name) : Node(name) {
    odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_footprint");
    // 创建 odom 话题订阅者，使用传感器数据的 Qos
    odom_subscribe_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS(),
        std::bind(&OdomTopic2TF::odom_callback_, this, std::placeholders::_1));
    // 创建一个tf2_ros::TransformBroadcaster用于广播坐标变换
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscribe_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  bool warned_empty_frame_once_ = false;
  // 回调函数，处理接收到的odom消息，并发布tf
  void odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = msg->header; // 使用消息的时间戳
    if (transform.header.frame_id.empty()) {
      transform.header.frame_id = odom_frame_id_;
    }

    if (msg->child_frame_id.empty()) {
      transform.child_frame_id = base_frame_id_;
      if (!warned_empty_frame_once_) {
        RCLCPP_WARN(this->get_logger(),
                    "Received /odom with empty child_frame_id, fallback to '%s'.",
                    base_frame_id_.c_str());
        warned_empty_frame_once_ = true;
      }
    } else {
      transform.child_frame_id = msg->child_frame_id; // 1.7.0 之前的固件，小车设置里要c_frameid改称base_footprint
    }

    transform.transform.translation.x = msg->pose.pose.position.x;
    transform.transform.translation.y = msg->pose.pose.position.y;
    transform.transform.translation.z = msg->pose.pose.position.z;
    transform.transform.rotation.x = msg->pose.pose.orientation.x;
    transform.transform.rotation.y = msg->pose.pose.orientation.y;
    transform.transform.rotation.z = msg->pose.pose.orientation.z;
    transform.transform.rotation.w = msg->pose.pose.orientation.w;
    // 广播坐标变换信息
    tf_broadcaster_->sendTransform(transform);
  };
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomTopic2TF>("odom2tf");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}