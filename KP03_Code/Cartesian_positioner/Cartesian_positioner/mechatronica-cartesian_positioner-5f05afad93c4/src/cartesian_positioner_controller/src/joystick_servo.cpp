// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include "detection_interfaces/srv/trigger_detection.hpp"
#include <map>

// We'll just set up parameters here
const std::string JOY_TOPIC = "/joy";
const std::string JOINT_STATE_TOPIC = "/joint_states";
const std::string X_JOINT_NAME = "x_joint_plan";
const std::string Y_JOINT_NAME = "y_joint";
const std::string Z_JOINT_NAME = "z_joint";
const std::string TWIST_TOPIC = "/servo_server/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_server/delta_joint_cmds";
const std::string DETECTIONS_SERVICE = "/joystick/measure_plant";
const size_t ROS_QUEUE_SIZE = 10;
const std::string EEF_FRAME_ID = "tool";
const std::string BASE_FRAME_ID = "root_link";
const double HALT_DETECTION_THRESHOLD = 0.02;
const double MOVE_DETECTION_THRESHOLD = 0.10;
const int TIMEOUT = 30000; //[ms]

// Enums for button names -> axis/button array index
enum Button
{
  X = 0,
  CIRCLE = 1,
  TRIANGLE = 2,
  SQUARE = 3,
  LEFT_BUMPER = 4,
  RIGHT_BUMPER = 5,
  CHANGE_VIEW = 6,
  MENU = 7,
  HOME = 8,
  LEFT_STICK_CLICK = 9,
  RIGHT_STICK_CLICK = 10,
  
  // constants to store active buttons
  DO_MEASURE = 4,
  DO_PLAY = 6
};


// For Google Stadia controller
// enum Axis
// {
//   LEFT_STICK_X = 0,
//   LEFT_STICK_Y = 1,
//   LEFT_TRIGGER = 5,
//   RIGHT_STICK_X = 2,
//   RIGHT_STICK_Y = 3,
//   RIGHT_TRIGGER = 4,
//   D_PAD_X = 6,
//   D_PAD_Y = 7
// };

//PS controller
enum Axis
{
  LEFT_STICK_X = 0,
  LEFT_STICK_Y = 1,
  LEFT_TRIGGER = 2,
  RIGHT_STICK_X = 3,
  RIGHT_STICK_Y = 4,
  RIGHT_TRIGGER = 5,
  D_PAD_X = 6,
  D_PAD_Y = 7
};

// Some axes have offsets (e.g. the default trigger position is 1.0 not 0)
// This will map the default values for the axes
std::map<Axis, double> AXIS_DEFAULTS = { { LEFT_TRIGGER, 1.0 }, { RIGHT_TRIGGER, 1.0 } };
std::map<Button, double> BUTTON_DEFAULTS;

// To change controls or setup a new controller, all you should to do is change the above enums and the follow 2
// functions
/** \brief // This converts a joystick axes and buttons array to a TwistStamped or JointJog message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param twist A TwistStamped message to update in prep for publishing
 * @param joint A JointJog message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a JointJog
 */
control_msgs::msg::JointJog convertJoyToCmd(const std::vector<float>& axes, bool& fine_control)
{
  control_msgs::msg::JointJog joint;

  // only use joint jogging
  float PI = 3.1415;

  float scaling = fine_control? 0.3 : 0.75;

  // Map the right stick to the X and Y 
  joint.joint_names.push_back("x_joint_plan");
  joint.velocities.push_back(axes[RIGHT_STICK_X]*scaling);
  joint.joint_names.push_back("y_joint");
  joint.velocities.push_back(-axes[RIGHT_STICK_Y]*scaling);

  // Map the left stick to the distal joints
  joint.joint_names.push_back("z_joint");
  joint.velocities.push_back(-axes[LEFT_STICK_Y]*scaling);
  joint.joint_names.push_back("a_joint");
  joint.velocities.push_back(axes[LEFT_STICK_X]*scaling*PI*2*2); //additional *2 to increase speed

  return joint;
}

control_msgs::msg::JointJog setZspeedOnly(double speed)
{
  control_msgs::msg::JointJog joint;

  joint.joint_names.push_back("z_joint");
  joint.velocities.push_back(speed);

  joint.header.frame_id = BASE_FRAME_ID;
  return joint;
}

using namespace std::chrono_literals;

namespace cartesian_positioner_controller
{

enum class TouchState {
  IDLE = 0,
  MOVING_DOWN = 1,
  WAITING_AT_WEED = 2,
  MOVING_UP = 3,
  MEASURE = 4,
  SWITCH_CONTROLLERS = 5
};

enum class ControlMode {
  MANUAL = 0,
  REPLAY = 1
};

class JoyToServoPub : public rclcpp::Node
{
public:
  JoyToServoPub(const rclcpp::NodeOptions& options)
    : Node("joy_to_twist_publisher", options), frame_to_publish_(BASE_FRAME_ID)
  {
    // Setup pub/sub
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        JOY_TOPIC, ROS_QUEUE_SIZE, std::bind(&JoyToServoPub::joyCB, this, std::placeholders::_1));
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        JOINT_STATE_TOPIC, ROS_QUEUE_SIZE, std::bind(&JoyToServoPub::jointStateCB, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, ROS_QUEUE_SIZE);
    joystick_state_pub_ = this->create_publisher<std_msgs::msg::String>("/joystick/state", ROS_QUEUE_SIZE);

    detection_client_ = this->create_client<detection_interfaces::srv::TriggerDetection>(DETECTIONS_SERVICE);
    controller_manager_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
    send_positions_client_ = this->create_client<std_srvs::srv::Trigger>("/joystick/send_list");

    // Create a service client to start the ServoServer
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_server/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    touch_timer_ = create_wall_timer(1ms, std::bind(&JoyToServoPub::touch_weed, this));

    Z_distance_mm_ = declare_parameter("delta_height_mm", Z_distance_mm_);
    time_at_weed_ = declare_parameter("seconds_at_weed", time_at_weed_);
    time_moving_ = declare_parameter("seconds_moving", time_moving_);

    RCLCPP_INFO(get_logger(), "X button press settings: moving down %d mm and waiting for %3.2f seconds", Z_distance_mm_, time_at_weed_);
  }

  void touch_weed()
  {
    float Z_speed = 1.0; // [m/s]
    int time_at_weed_ms = time_at_weed_ * 1000;

    int time_moving_ms = time_moving_ * 1000;

    bool halted = abs(joint_velocities_[Z_JOINT_NAME]) < HALT_DETECTION_THRESHOLD;
    if (abs(joint_velocities_[Z_JOINT_NAME]) > MOVE_DETECTION_THRESHOLD)
    {
      movingZ_ = true;
    }

    switch(touch_state_){
      case TouchState::IDLE:
      { 
        movingZ_ = false;
        halted = false;
        break;
      }
      case TouchState::SWITCH_CONTROLLERS:
      {
        if (switching_call_.wait_for(500ms) == std::future_status::ready)
        {
          auto result = switching_call_.get();
          RCLCPP_INFO(get_logger(), "Successfully switched controllers");
          touch_step = 0;
          auto msg = std::make_unique<std_msgs::msg::String>();
        
          if (mode_ == ControlMode::MANUAL)
          {
            RCLCPP_INFO(get_logger(), "ControlMode 'MANUAL'");
            msg->data = "MANUAL";
          }else if (mode_ == ControlMode::REPLAY)
          {
            RCLCPP_INFO(get_logger(), "ControlMode 'REPLAY'");
            msg->data = "REPLAY";
            send_positions_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
          }
          joystick_state_pub_->publish(std::move(msg));
          touch_state_ = TouchState::IDLE;
        }else if (touch_step > TIMEOUT)
        {
          RCLCPP_WARN(get_logger(), "Controller switching service did not provide timely response, please check controller manager node");
          touch_state_ = TouchState::IDLE;
        }
        touch_step++;
        break;
      }
      case TouchState::MEASURE:
      {
        if (measurement_call_.wait_for(50ms) == std::future_status::ready)
        {
          auto detection = measurement_call_.get();
          RCLCPP_INFO(get_logger(), "Received position XYZ=[%3.2f, %3.2f, %3.2f]", detection->position.x, detection->position.y, detection->position.z);
          touch_step = 0;
          touch_state_ = TouchState::IDLE;
        }else if (touch_step > TIMEOUT)
        {
          RCLCPP_WARN(get_logger(), "Measurement service did not provide timely response, please check node with plant measurement service");
          touch_state_ = TouchState::IDLE;
        }
        touch_step++;
        break;
      }
      case TouchState::MOVING_DOWN:
      {
        auto joint = std::make_unique<control_msgs::msg::JointJog>(setZspeedOnly(Z_speed));
        joint->header.stamp = this->now();
        joint_pub_->publish(std::move(joint));
        //if (movingZ_ && (halted || joint_positions_[Z_JOINT_NAME] >= (z_start_position_ + (Z_distance_mm_/1000.0)))) //TODO optional use absolute positioning?
        //{
        touch_step++;
        if (touch_step >= time_moving_ms)
        {
          //RCLCPP_INFO(get_logger(), "Moving down done [halted=%d, movingZ=%d]", halted, movingZ_);
          movingZ_ = false;
          touch_step = 0;
          touch_state_ = TouchState::WAITING_AT_WEED;
        }else if (touch_step > TIMEOUT)
        {
          movingZ_ = false;
          touch_step = 0;
          touch_state_ = TouchState::IDLE;
          RCLCPP_INFO(get_logger(), "X button movement timed-out");
        }
        break;
      }
      case TouchState::WAITING_AT_WEED:
      {
        auto joint = std::make_unique<control_msgs::msg::JointJog>(setZspeedOnly(0));
        joint->header.stamp = this->now();
        joint_pub_->publish(std::move(joint));
        touch_step++;
        if (touch_step >= time_at_weed_ms)
        {
          RCLCPP_INFO(get_logger(), "Waiting at weed done [halted=%d, movingZ=%d]", halted, movingZ_);
          touch_step = 0;
          touch_state_ = TouchState::MOVING_UP;
          movingZ_ = false;
        }
        break;
      }
      case TouchState::MOVING_UP:
      {
        auto joint = std::make_unique<control_msgs::msg::JointJog>(setZspeedOnly(-Z_speed));
        joint->header.stamp = this->now();
        joint_pub_->publish(std::move(joint));
        //if (movingZ_ && (halted || joint_positions_[Z_JOINT_NAME] <= z_start_position_))
        //{
        touch_step++;
        if (touch_step >= time_moving_ms)
        {  
          //set speed to zero  
          auto joint = std::make_unique<control_msgs::msg::JointJog>(setZspeedOnly(0));
          joint->header.stamp = this->now();
          joint_pub_->publish(std::move(joint));
          //RCLCPP_INFO(get_logger(), "X button movement done [halted=%d, movingZ=%d]", halted, movingZ_);
          movingZ_ = false;
          touch_step = 0;
          touch_state_ = TouchState::IDLE;         
        }
        break;
      }
      default:
      {
        RCLCPP_INFO(get_logger(), "Unknown state");
        break;
      }
    }
  }

  void jointStateCB(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i=0; i < msg->name.size(); i++)
    {
      joint_positions_[msg->name[i]] = msg->position[i];
      joint_velocities_[msg->name[i]] = msg->velocity[i];
    }
  }

  void joyCB(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (touch_state_ != TouchState::IDLE)
    {
      //don't process button events when touching weeds, set all active keys to false
      active_buttons_ = {false, false, false, false, false, false, false};
      return;
    }

    if (msg->buttons[X] && mode_ == ControlMode::MANUAL) //trigger weed touching on button PRESS
    {
      RCLCPP_INFO(get_logger(), "X button pressed: moving down %d mm and waiting for %3.2f seconds", Z_distance_mm_, time_at_weed_);
      z_start_position_ = joint_positions_[Z_JOINT_NAME];
      touch_state_ = TouchState::MOVING_DOWN;
    }

    if (msg->buttons[TRIANGLE]){
      active_buttons_.at(TRIANGLE) = true; //store event
    }
    if (msg->buttons[CIRCLE]){
      active_buttons_.at(CIRCLE) = true; //store event
    }
    if (msg->buttons[RIGHT_BUMPER]){
      active_buttons_.at(RIGHT_BUMPER) = true; //store event
    }
    if (msg->axes[D_PAD_X] == 1.0){
      active_buttons_.at(DO_MEASURE) = true; //store event
    }
    if (msg->axes[D_PAD_X] == -1.0){
      active_buttons_.at(DO_PLAY) = true; //store event
    }

    if (msg->axes[D_PAD_X] == 0.0 && active_buttons_[DO_MEASURE]){
      RCLCPP_INFO(get_logger(), "Switching to 'measurement' mode");
      switchControllers(true);
      touch_state_ = TouchState::SWITCH_CONTROLLERS;
      mode_ = ControlMode::MANUAL;
      active_buttons_ = {false, false, false, false, false, false, false};
    }
    else if (msg->axes[D_PAD_X] == 0.0 && active_buttons_[DO_PLAY]){
      RCLCPP_INFO(get_logger(), "Switching to 'replay' mode");
      switchControllers(false);
      touch_state_ = TouchState::SWITCH_CONTROLLERS;
      mode_ = ControlMode::REPLAY;
      active_buttons_ = {false, false, false, false, false, false, false};
    }
    else if (mode_ == ControlMode::MANUAL)
    {
      if ((!msg->buttons[TRIANGLE] && active_buttons_[TRIANGLE]) || (!msg->buttons[CIRCLE] && active_buttons_[CIRCLE]) ) //measure plants with tool and joystick (trigger on button RELEASE)
      {
        if (detection_client_->wait_for_service(1s))
        {
          auto trigger = std::make_shared<detection_interfaces::srv::TriggerDetection::Request>();
          trigger->type = active_buttons_[TRIANGLE]? "crop" : "weed";
          RCLCPP_WARN(get_logger(), "Requesting position with label '%s'...",trigger->type.c_str());
          measurement_call_ = detection_client_->async_send_request(trigger);
          touch_state_ = TouchState::MEASURE;
        }
        else
        {
          RCLCPP_WARN(get_logger(), "Measurement service unavailable, please start node with plant measurement service");
        }
        //reset active buttons
        active_buttons_ = {false, false, false, false, false, false, false};
      }
      else if (!msg->buttons[RIGHT_BUMPER] && active_buttons_[RIGHT_BUMPER])
      {
        fine_control_ = !fine_control_;
        //reset active buttons
        active_buttons_ = {false, false, false, false, false, false, false};
        if (fine_control_)
        {
          RCLCPP_INFO(get_logger(), "Toggling fine control to ON");
        }else
        {
          RCLCPP_INFO(get_logger(), "Toggling fine control to OFF");
        }
      }
      else
      {
        // Convert the joystick message to JointJog and publish
        auto joint_msg = std::make_unique<control_msgs::msg::JointJog>(convertJoyToCmd(msg->axes, fine_control_));
          
        // publish the JointJog
        joint_msg->header.stamp = this->now();
        joint_msg->header.frame_id = BASE_FRAME_ID;
        joint_pub_->publish(std::move(joint_msg));
      }
    }

  }

  void switchControllers(bool set_manual){
    auto request = std::make_shared< controller_manager_msgs::srv::SwitchController::Request>();
    
    if (set_manual)
    {
      request->start_controllers = manual_controllers;
      request->stop_controllers = trajectory_controllers;
    }
    else
    {
      request->start_controllers = trajectory_controllers;
      request->stop_controllers = manual_controllers;
    }
    request->strictness = request->BEST_EFFORT;
    switching_call_ = controller_manager_client_->async_send_request(request);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr joystick_state_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
  
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr send_positions_client_;
  rclcpp::Client<detection_interfaces::srv::TriggerDetection>::SharedPtr detection_client_;  
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_manager_client_;
  rclcpp::Client<detection_interfaces::srv::TriggerDetection>::SharedFuture measurement_call_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture switching_call_;

  rclcpp::TimerBase::SharedPtr touch_timer_;
  std::vector<bool> active_buttons_ = {false, false, false, false, false, false, false}; //track buttons states to process key releases instead of presses
  
  std::vector<std::string> manual_controllers = {"joystick_position_controller"};
  std::vector<std::string> trajectory_controllers = {"manipulator_joint_trajectory_controller"};

  TouchState touch_state_ = TouchState::IDLE;
  ControlMode mode_ = ControlMode::MANUAL;

  int touch_step = 0;
  std::string frame_to_publish_;

  double z_start_position_ = 0;
  std::map<std::string, double> joint_positions_;
  std::map<std::string, double> joint_velocities_;

  bool fine_control_ = false;
  bool movingZ_ = false;

  //parameters
  int Z_distance_mm_ = 100; //[mm]
  float time_at_weed_ = 1.0; //[s]
  float time_moving_ = 0.1; //[s]

};  // class JoyToServoPub

}  // namespace moveit_servo

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cartesian_positioner_controller::JoyToServoPub)