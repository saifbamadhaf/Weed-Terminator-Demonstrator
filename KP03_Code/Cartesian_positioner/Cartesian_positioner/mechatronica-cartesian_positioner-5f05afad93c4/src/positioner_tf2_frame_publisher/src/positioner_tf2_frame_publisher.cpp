#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "detection_interfaces/msg/conveyor.hpp"

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("positioner_tf2_frame_publisher")
  {

    topicname_ = this->declare_parameter<std::string>("topicname", "encoder");

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
    // callback function on each message
    subscription_ = this->create_subscription<detection_interfaces::msg::Conveyor>(
      topicname_, 10,
      std::bind(&FramePublisher::handle_conveyor_motion, this, std::placeholders::_1));

      //set initial position
      detection_interfaces::msg::Conveyor msg;
      msg.speed = 0;
      msg.distance_travelled = 0;
      msg.turns = 0;
      handle_conveyor_motion(std::make_shared<detection_interfaces::msg::Conveyor>(msg));
  }

private:
  void handle_conveyor_motion(const std::shared_ptr<detection_interfaces::msg::Conveyor> msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = "world";
    t.child_frame_id = "root_link";

    t.transform.translation.x = -msg->distance_travelled;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<detection_interfaces::msg::Conveyor>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string topicname_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}