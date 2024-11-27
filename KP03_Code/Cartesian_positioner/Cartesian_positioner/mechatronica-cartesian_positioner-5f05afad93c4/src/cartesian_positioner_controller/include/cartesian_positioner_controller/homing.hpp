#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"

using namespace std::chrono_literals;

#define DEFAULT_BASE_SPEED 5.0 //[rad/s]
#define DEFAULT_RETRACTION_TIME 5.0 //[s]
#define DEFAULT_POSITION_TOLERANCE 0.1 //[rad]
#define DEFAULT_SPEED_TOLERANCE 0.2 //[rad/s]
#define DEFAULT_TORQUE_LIMIT 0.5 //[Nm]

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class HomingNode : public rclcpp::Node
{

  enum class HomingState {
    Init, 
    Error,
    Done,
    Move_min_first,
    Retract,
    Stop_retract,
    Move_min_second,
    Move_max,
  };

  public:
    HomingNode();

  private:
    //parameters
    float expected_travel_;
    std::string joint_name_;

    float base_speed_;
    std::chrono::duration<float> retraction_time_;
    float position_tolerance_;
    float speed_tolerance_;

    //measurements
    float joint_velocity_;
    float min_position_;
    float max_position_;
    
    //state variables
    HomingState state = HomingState::Init;
    int direction_ = 0;
    bool joint_moving_ = false;

    //object-pointers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr retract_timer_;
    rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

    //functions
    void update_velocity();
    void state_callback(const control_msgs::msg::DynamicJointState::SharedPtr msg);
    void update_state(float current_position = -1);
    void retract_timer_callback();

};
