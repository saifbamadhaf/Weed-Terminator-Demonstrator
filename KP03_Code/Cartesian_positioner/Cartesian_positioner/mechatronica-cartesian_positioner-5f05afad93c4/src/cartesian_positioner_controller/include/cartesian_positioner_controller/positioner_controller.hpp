// Includes
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/collision_object.hpp>

#include <rviz_visual_tools/rviz_visual_tools.hpp>

#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <ctime>
#include <chrono>
#include <iostream>
#include <memory>
#include <deque>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include <control_msgs/msg/joint_jog.hpp>
#include "detection_interfaces/msg/detections.hpp"
#include "detection_interfaces/msg/detected.hpp"
#include "detection_interfaces/msg/conveyor.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

//constants

const std::string JOINT_STATES_TOPIC = "/joint_states";
static const std::string PLANNING_GROUP = "manipulator";
const std::string JOINT_TOPIC = "/servo_server/manipulator_joint_cmds";
const size_t ROS_QUEUE_SIZE = 10;
const float THRESHOLD_X_PLANNING = 0.15; //start planning when weeds are moved beyond this x position
const float MOVE_BACK_THRESHOLD = 0.85;
const float Z_MOVING_UP = 0.05;
const float CONE_DELTA_H = 0.03;
const float CONE_HEIGHT = 0.17;
const float CONE_RADIUS = 0.08;
const float TIMEOUT = 7;

const float START_LINE = -1.755;

//scoring (gamification)
const std::string CROP_TOUCH_MSG = "crop";
const std::string WEED_TOUCH_MSG = "weed";
const float R_TOUCH_THRESHOLD = 0.01;
const float Z_TOUCH_THRESHOLD = 0.01;

const float _COMP_VEL_FACTOR_HACK_ = 1.0;
const float max_velocity_scaling = 0.5;
const float max_acceleration_scaling= 0.5;

float back_vel = -0.40; // negative // Velocity with which to move back to the starting point
float time_at_weed = 1.0; // Time to stay at the crop once it's reached
float cone_height_ = CONE_HEIGHT + CONE_DELTA_H;
float cone_radius_ = CONE_RADIUS;
bool use_z_weed_position = true;
bool use_visual_tools_ = false;
float fixed_z_weed_position = -0.1;

enum class PositionerState {
  PLAN = 0,
  EXECUTING = 1,
  WAITING_AT_WEED = 2,
  MOVE_UP = 3,
  MOVING_UP = 4,
  MOVE_BACK = 5,
  MOVING_BACK = 6,
  HOMED = 7,
  IDLE = 8,
  MANUAL = 9,
  RECOVERY = 10
};

struct CompareDistanceWeeds
{
  CompareDistanceWeeds(geometry_msgs::msg::Point current_position)
  {
    this->current_position = current_position;
  }
  bool operator ()(std::vector<double> p1, std::vector<double> p2)
  {
    return sqrt(pow(p1.at(0)-current_position.x,2)+pow(p1.at(1)-current_position.y,2)) < sqrt(pow(p2.at(0)-current_position.x,2)+pow(p2.at(1)-current_position.y,2));
  }
  geometry_msgs::msg::Point current_position;
};

struct CompareDistanceCrops
{
  CompareDistanceCrops(geometry_msgs::msg::Point current_position)
  {
    this->current_position = current_position;
  }
  bool operator ()(std::pair<std::string, geometry_msgs::msg::Pose> p1, std::pair<std::string, geometry_msgs::msg::Pose> p2)
  {
    return sqrt(pow(p1.second.position.x-current_position.x,2)+pow(p1.second.position.y-current_position.y,2)) < sqrt(pow(p2.second.position.x-current_position.x,2)+pow(p1.second.position.y-current_position.y,2));
  }
  geometry_msgs::msg::Point current_position;
};

class PositionerController : public rclcpp::Node
{
  public:
    PositionerController();
    void init();

  private:
    //functions   
    void add_crop(std::vector<double> crop_location);
    bool check_weed_in_range();
    void move_crops(double delta);
    void conveyor_callback(detection_interfaces::msg::Conveyor::SharedPtr conveyor_msg);
    void detections_callback(detection_interfaces::msg::Detections::SharedPtr msg);
    void control_callback();
    void joint_state_callback(sensor_msgs::msg::JointState::SharedPtr msg);
    void execution_callback(std_msgs::msg::String::SharedPtr msg);
    void joystick_state_callback(std_msgs::msg::String::SharedPtr msg);
    void visualize_end_effector(geometry_msgs::msg::PoseStamped current_pose, rviz_visual_tools::Colors color = rviz_visual_tools::Colors::RED);
    bool pose_at_target(geometry_msgs::msg::PoseStamped current_pose, double tolerance = 0.01);
    void check_publish_interaction(geometry_msgs::msg::PoseStamped current_pose);

    //variables
    PositionerState state_ = PositionerState::IDLE;
    PositionerState old_state_ = PositionerState::IDLE;
    PositionerState joystick_state_request_ = PositionerState::IDLE;

    std::deque<std::vector<double>> weeds_to_destroy;
    std::deque<std::vector<double>> crops_to_add;
    std::map<std::string, geometry_msgs::msg::Pose> active_collision_objects; //maps pose-coordinate to id to be able to remove crop when it is passed
    
    std::vector<double> weed_position;
    std::mutex m_weeds;
    std::mutex m_crops;

    int n_weeds_detected=0;
    int n_weeds_destroyed=0;
    
    std::vector<std::string> crop_ids;
    std::vector<double> joint_states_;
    int crop_count = 0;
    float comp_vel = 0; // Compensation velocity
    float prev_comp_pos = 0; //previous position of compensation joint
    double current_x_position; //relative to world (travelled distance)

    double start_x; //relative to world (start of plan)
    double last_x; //relative to world (travelled distance)

    const std::string JOINT_TOPIC = "/servo_server/manipulator_joint_cmds";
    const std::string JOINT_STATES_TOPIC = "/joint_states";
    const size_t ROS_QUEUE_SIZE = 10;

    // For visualizing things in rviz
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
        
    moveit::planning_interface::MoveGroupInterface::Plan plan_;

    //rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tool_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr touch_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr execution_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr joystick_state_sub_;
    rclcpp::Subscription<detection_interfaces::msg::Conveyor>::SharedPtr conveyor_sub_;
    rclcpp::Subscription<detection_interfaces::msg::Detections>::SharedPtr detections_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    const moveit::core::JointModelGroup* joint_model_group;
    bool enableCompensation = false;
    geometry_msgs::msg::PoseStamped target_pose;

    std::chrono::time_point<std::chrono::system_clock> start_time;
    std::chrono::time_point<std::chrono::system_clock> state_start_time;
    std::chrono::time_point<std::chrono::system_clock> current_time;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    std::string state_to_string(PositionerState state)
    {
      switch(state)
      {
        case PositionerState::MANUAL: return "MANUAL";
        case PositionerState::IDLE: return "IDLE";
        case PositionerState::PLAN: return "PLAN";
        case PositionerState::EXECUTING: return "EXECUTING";
        case PositionerState::WAITING_AT_WEED: return "WAITING_AT_WEED";
        case PositionerState::MOVE_UP: return "MOVE_UP";
        case PositionerState::MOVING_UP: return "MOVING_UP";
        case PositionerState::MOVE_BACK: return "MOVE_BACK";
        case PositionerState::MOVING_BACK: return "MOVING_BACK";
        case PositionerState::HOMED: return "HOMED";
        case PositionerState::RECOVERY: return "RECOVERY";
        default: return "<unknown state>";
      }
    }
};