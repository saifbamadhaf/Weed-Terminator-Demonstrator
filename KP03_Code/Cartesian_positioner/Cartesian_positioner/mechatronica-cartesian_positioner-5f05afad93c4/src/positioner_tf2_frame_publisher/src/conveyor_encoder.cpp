#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "detection_interfaces/msg/conveyor.hpp"

using namespace std::chrono_literals;

class ConveyorPublisher : public rclcpp::Node
{
  public:
    ConveyorPublisher()
    : Node("conveyor_publisher"), count_(0), distance_(0.0), speed_(0.0)
    {
      speed_ = declare_parameter("belt_speed", 0.0);
      publish_period_ = declare_parameter("publish_period_ms", 20);
      belt_length_ = declare_parameter("belt_length", 3.0);
      publish_zero_ = declare_parameter("publish_zero", false);

      double published_speed = publish_zero_ ? 0.0 : speed_;
      RCLCPP_INFO(get_logger(), "Starting simulation of conveyor belt with length=%4.3fm and speed=%3.2fm/s. Publishing updates every %d ms", belt_length_, published_speed, publish_period_);
      if (publish_zero_)
      {  
        RCLCPP_INFO(get_logger(), "Increasing turns as if belt is moving with speed=%3.2fm/s", speed_);
      }  
      publisher_ = this->create_publisher<detection_interfaces::msg::Conveyor>("conveyor", 10);
      timer_ = this->create_wall_timer(1ms*publish_period_, std::bind(&ConveyorPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      if (!indexed && distance_ > 0.1*belt_length_)
      {
        distance_ = 0;
        indexed = true;
      }
      distance_ += speed_*(publish_period_/1000.0);
      auto message = detection_interfaces::msg::Conveyor();
      message.header.stamp = this->get_clock()->now();
      message.header.stamp.nanosec+=50000000;
      message.speed = publish_zero_ ? 0.0 : speed_;
      message.distance_travelled = publish_zero_ ? 0.0 : distance_;
      message.turns = int(distance_ / belt_length_) + ((indexed)? 1 : 0);
      publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<detection_interfaces::msg::Conveyor>::SharedPtr publisher_;
    size_t count_;
    double distance_ = 0.0;
    double speed_;
    int publish_period_;
    double belt_length_;
    bool publish_zero_ = false;
    bool indexed = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConveyorPublisher>());
    rclcpp::shutdown();
  return 0;
}