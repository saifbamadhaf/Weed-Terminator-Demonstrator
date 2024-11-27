#include "cartesian_positioner_controller/positioner_controller.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

PositionerController::PositionerController() : Node("positioner_controller")
{
  //initialize publishers/subscribers
  //joint_pub_ = create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, ROS_QUEUE_SIZE);
  joint_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/x_joint_comp_position_controller/commands", ROS_QUEUE_SIZE);
  tool_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/tool_pose", ROS_QUEUE_SIZE);
  touch_publisher_ = create_publisher<std_msgs::msg::String>("/touches", ROS_QUEUE_SIZE);
  debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/debug", ROS_QUEUE_SIZE);
  detections_sub_ = create_subscription<detection_interfaces::msg::Detections>("/filtered_detections", ROS_QUEUE_SIZE, std::bind(&PositionerController::detections_callback, this, _1));
  conveyor_sub_ = create_subscription<detection_interfaces::msg::Conveyor>("/conveyor", ROS_QUEUE_SIZE, std::bind(&PositionerController::conveyor_callback, this, _1));
  execution_sub_ = create_subscription<std_msgs::msg::String>("/trajectory_execution_event", ROS_QUEUE_SIZE, std::bind(&PositionerController::execution_callback, this, _1));
  joystick_state_sub_ = create_subscription<std_msgs::msg::String>("/joystick/state", ROS_QUEUE_SIZE, std::bind(&PositionerController::joystick_state_callback, this, _1));

  auto start_in_manual = declare_parameter("manual", false);

  use_visual_tools_ = declare_parameter("display_markers", use_visual_tools_);

  use_z_weed_position = declare_parameter("use_measured_z", use_z_weed_position);
  fixed_z_weed_position = declare_parameter("z_position_ground", fixed_z_weed_position);

  time_at_weed = declare_parameter("seconds_at_weed", time_at_weed);
  float cone_height_above_ground = declare_parameter("cone_height", CONE_HEIGHT);
  cone_radius_ = declare_parameter("cone_radius", CONE_RADIUS);
  cone_height_ = cone_height_above_ground + CONE_DELTA_H;

  if (use_visual_tools_)
  {
    //initialize visualization functions
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world","/rviz_visual_markers", this));
    visual_tools_->enableBatchPublishing();
    visual_tools_->enableFrameLocking();
    visual_tools_->deleteAllMarkers();
    visual_tools_->trigger();
  }
  if (start_in_manual){
    state_ = PositionerState::MANUAL;
    joystick_state_request_ = PositionerState::MANUAL;
  }
}

void PositionerController::init()
{
  //initialize move_group interface
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);

  RCLCPP_INFO(get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());

  RCLCPP_INFO(get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());

  RCLCPP_INFO(get_logger(), "Reference frame: %s", move_group_->getPoseReferenceFrame().c_str());

  move_group_->setMaxVelocityScalingFactor(max_velocity_scaling);
  move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling);

  control_timer_ = create_wall_timer(20ms, std::bind(&PositionerController::control_callback, this));
  
  joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(JOINT_STATES_TOPIC, ROS_QUEUE_SIZE, std::bind(&PositionerController::joint_state_callback, this, _1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (!tf_buffer_->canTransform("a_stage", "base_link", tf2::TimePointZero, 10s))
  {
    RCLCPP_ERROR(get_logger(), "Transform unavailable");
  }
}

void PositionerController::conveyor_callback(detection_interfaces::msg::Conveyor::SharedPtr conveyor_msg){
  comp_vel = conveyor_msg->speed;
  current_x_position = conveyor_msg->distance_travelled;
}

void PositionerController::execution_callback(std_msgs::msg::String::SharedPtr msg){
  RCLCPP_INFO(get_logger(),"Received execution event: %s", msg->data);
}

void PositionerController::joystick_state_callback(std_msgs::msg::String::SharedPtr msg){
  RCLCPP_INFO(get_logger(),"Received joystick state change request: %s", msg->data.c_str());
  if (msg->data == "MANUAL")
  {
    joystick_state_request_ = PositionerState::MANUAL;
  }
  else
  {
    joystick_state_request_ = PositionerState::MOVE_UP;
  }
}

void PositionerController::detections_callback(detection_interfaces::msg::Detections::SharedPtr msg){

  int n_crops=0, n_weeds=0;

  std::lock_guard<std::mutex> lck_crops(m_crops);
  std::lock_guard<std::mutex> lck_weeds(m_weeds);
  for(auto detection : msg->detected){
    float z_pos = detection.position.z;
    if (!use_z_weed_position)
    {
      z_pos = fixed_z_weed_position;
    }

    if(detection.type == "crop"){
      std::vector<double> location{detection.position.x, detection.position.y, z_pos};
      crops_to_add.push_back(location);
      n_crops++;
    }

    if(detection.type == "weed"){
      std::vector<double> location{detection.position.x, detection.position.y, z_pos};
      weeds_to_destroy.push_back(location);
      n_weeds++;
    }
  }
  n_weeds_detected += n_weeds;
  if (use_visual_tools_) visual_tools_->trigger();
  RCLCPP_INFO(get_logger(), "Detections received: %d crops and %d weeds", n_crops, n_weeds);
}

void PositionerController::add_crop(std::vector<double> crop_location){
  
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_->getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "crop_" + std::to_string(crop_count);

  // Define a box to add to the world.
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.CONE;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.CONE_HEIGHT] = cone_height_;
  primitive.dimensions[primitive.CONE_RADIUS] = cone_radius_;
  
  geometry_msgs::msg::Pose crop_pose;

  tf2::Quaternion q;
  q.setEuler(3.14159, 0, 0);

  // Define a pose for the crop (specified relative to frame_id).

  crop_pose.position.x = crop_location.at(0);
  crop_pose.position.y = crop_location.at(1);
  crop_pose.position.z = crop_location.at(2) + cone_height_ / 2 - CONE_DELTA_H;

  crop_pose.orientation.x = q.x();
  crop_pose.orientation.y = q.y();
  crop_pose.orientation.z = q.z();
  crop_pose.orientation.w = q.w();

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(crop_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  planning_scene_interface_.addCollisionObjects(collision_objects);

  //add to active crops for later removal from scene
  active_collision_objects[collision_object.id] = crop_pose;
  RCLCPP_DEBUG(get_logger(), "Crop '%s' added at x= %4.3f", collision_object.id.c_str(), crop_pose.position.x);
  crop_count++;

}

bool PositionerController::check_weed_in_range()
{
  bool planToMove = false;
  bool firstOutOfRange = false;
  std::lock_guard<std::mutex> lck(m_weeds);
  while (!firstOutOfRange && !planToMove && !weeds_to_destroy.empty())
  {
    weed_position = weeds_to_destroy.front();
    firstOutOfRange = (weed_position.at(0)+start_x < THRESHOLD_X_PLANNING); // use start_x because planning frame might be displaced by x_comp axis
    /* if (firstOutOfRange)
    {
      RCLCPP_INFO(get_logger(), "First weed in list not yet within reach (x=%3.2f)", weed_position.at(0)+current_x_position);
    } */
    if (!firstOutOfRange)
    {
      weeds_to_destroy.pop_front();
      planToMove = (weed_position.at(0)+current_x_position < MOVE_BACK_THRESHOLD); //skip weed if out of range
    }
  }
  return planToMove;
}

void PositionerController::move_crops(double delta)
{    
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  for (auto &obj : active_collision_objects)
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_->getPlanningFrame();
    obj.second.position.x += delta;
    collision_object.id = obj.first;
    collision_object.primitive_poses.push_back(obj.second);
    collision_object.operation = collision_object.MOVE;
    collision_objects.push_back(collision_object);
  }
  
  planning_scene_interface_.addCollisionObjects(collision_objects);
}

void PositionerController::joint_state_callback(sensor_msgs::msg::JointState::SharedPtr msg){
  joint_states_ = msg->position;
}

void PositionerController::check_publish_interaction(geometry_msgs::msg::PoseStamped current_pose)
{
  //check for crop collisions
  if (!active_collision_objects.empty()){
    auto nearest_crop = std::min_element(active_collision_objects.begin(), active_collision_objects.end(), CompareDistanceCrops(current_pose.pose.position));
    double dR = sqrt(pow(nearest_crop->second.position.x-current_pose.pose.position.x,2)+pow(nearest_crop->second.position.y-current_pose.pose.position.y,2));
    double dZ = abs(current_pose.pose.position.z-nearest_crop->second.position.z);
    double height_factor = dZ / cone_height_;
    if (dR < height_factor*cone_radius_ && dZ < cone_height_)
    {
      auto msg = std::make_unique<std_msgs::msg::String>();
      msg->data = CROP_TOUCH_MSG;
      touch_publisher_->publish(std::move(msg));
    }
  }

  //check for weed touches
  if (!weeds_to_destroy.empty()){
    
    auto nearest_weed = std::min_element(weeds_to_destroy.begin(), weeds_to_destroy.end(), CompareDistanceWeeds(current_pose.pose.position));

    if (sqrt(pow(nearest_weed->at(0)-current_pose.pose.position.x,2)+pow(nearest_weed->at(1)-current_pose.pose.position.y,2))< R_TOUCH_THRESHOLD && abs(current_pose.pose.position.z-nearest_weed->at(2)) < Z_TOUCH_THRESHOLD)
    {
      auto msg = std::make_unique<std_msgs::msg::String>();
      msg->data = WEED_TOUCH_MSG;
      touch_publisher_->publish(std::move(msg));
    }
  }

}

void PositionerController::control_callback(){
  std::vector<double> crop_location;
  std::chrono::duration<double> elapsed_time;

  if (joint_states_.empty())
  {
    RCLCPP_DEBUG(get_logger(), "Joint states not yet received");
    return;
  }

  //debug logging for state transitions
  if (state_ != old_state_)
  {
    state_start_time = std::chrono::system_clock::now();;
    RCLCPP_INFO(get_logger(), "State transition from '%s' to '%s'", state_to_string(old_state_).c_str(), state_to_string(state_).c_str());
  }
  old_state_ = state_;

  float actual_x_pos = joint_states_[1] + joint_states_[2];
  
  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.header.frame_id = "tool";
  if (!tf_buffer_->canTransform("tool", "root_link", tf2::TimePointZero, 1s))
  {
    RCLCPP_DEBUG(get_logger(), "Unable to obtain end-effector pose");
    return;
  }
  current_pose = tf_buffer_->transform(current_pose, "root_link");

  //abort current state and move up when nearing end of positioner frame
  if (actual_x_pos > MOVE_BACK_THRESHOLD && state_ != PositionerState::MANUAL && state_ != PositionerState::MOVING_UP && state_ != PositionerState::MOVE_BACK && state_ != PositionerState::MOVING_BACK){
    move_group_->stop();
    state_ = PositionerState::MOVE_UP;
  }
  
  auto weed_pose = current_pose;
  weed_pose.pose.position.x = START_LINE - weed_pose.pose.position.x + current_x_position;
  tool_publisher_->publish(weed_pose);
  
  switch(state_){
    case PositionerState::MANUAL:
    { 
      enableCompensation = false;
      check_publish_interaction(current_pose); //check for weed/crop touches 
      if (joystick_state_request_ != PositionerState::MANUAL)
      {
        state_ = joystick_state_request_;
      }
      break;
    }
    case PositionerState::HOMED:
    case PositionerState::IDLE:
    {
      if (joystick_state_request_ == PositionerState::MANUAL)
      {
        state_ = PositionerState::MANUAL;
        break;
      }
      
      //move existing collision objects
      enableCompensation = false;
      if (state_ == PositionerState::HOMED)
      {
        start_x = current_x_position;
        prev_comp_pos = 0;
      }
      if (current_x_position != last_x)
      {
        move_crops(current_x_position-last_x);
      }
      last_x = current_x_position;

      { //scope for lock
        std::lock_guard<std::mutex> lck(m_crops);
        while(!crops_to_add.empty())
        {
          crop_location = crops_to_add.front();
          crops_to_add.pop_front();
          crop_location.at(0)+=current_x_position;
          add_crop(crop_location);
        }
      }

      //check for crops to remove from scene
      for (auto &obj : active_collision_objects)
      {
        if (obj.second.position.x > 1.0)
        {
          crop_ids.push_back(obj.first);
        }
      }

      if (!crop_ids.empty())
      {
        planning_scene_interface_.removeCollisionObjects(crop_ids);
        RCLCPP_INFO(get_logger(), "Removed %d crops from scene (already passed)", crop_ids.size());
        for (auto &id : crop_ids)
        {
          active_collision_objects.erase(id);
        }
        crop_ids.clear();
      }
        
      if (check_weed_in_range())
      {
        state_ = PositionerState::PLAN;
      }else{
        if (state_ == PositionerState::IDLE)
        { //no need to move back if already HOMED
          state_ = PositionerState::MOVE_BACK;
        }
      }
      break;
    }
    case PositionerState::PLAN:
    { 
      enableCompensation = true;
      // When tool is already at an actual_x_pos > 0, compensation set point should continue at current x_comp
      // therefore subtract joint_states_[1] (x_comp value) to the current_x_position, such that the compensation starts at
      // x_comp = current_x_position - start_x 
      // ==> x_comp = current_x_position - current_x_position + joint_states_[1] = joint_states_[1]
      // TODO isn't this the same as not updating start_x at all?? 
      //                                <-----------Robot----------->
      //--conveyor belt----------------|----------|-|<-actual_x_pos->|----------------------
      //                               |          | |                |
      //                               |          |o|tool  *         |
      //                               |   *      | | c              |
      //                               |         c| |         *      |
      //-------------------------------|----------|-|----------------|----------------------
      //       |                           |     |    |    |  |      x0,robot
      //       x0,belt                     x,w1  x,c1 x,c2 x,w2 x,w3 current_x_position

      //start_x = current_x_position-joint_states_[1];
      //-joint_states_[1]???

      //keep planning from static, compensation is done by virtual x_link so planning positions should relate to start_x instead of current_x_position
      RCLCPP_INFO(get_logger(), "StartX=%3.2f", start_x);
      RCLCPP_INFO(get_logger(), "Moving to [%3.2f,%3.2f,%3.2f]", weed_position.at(0)+start_x, weed_position.at(1), weed_position.at(2));
      move_group_->clearPoseTargets();
      move_group_->setStartStateToCurrentState();
      move_group_->setPositionTarget(weed_position.at(0)+start_x, weed_position.at(1), weed_position.at(2));
      move_group_->setGoalTolerance(1e-3);
      target_pose = move_group_->getPoseTarget();
      move_group_->asyncMove();
      //use plan and asyncExecute to obtain planning result
      // if (move_group_->plan(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      // {
      //   target_pose = move_group_->getPoseTarget();
      //   move_group_->asyncExecute(plan_);
      state_ = PositionerState::EXECUTING;
      // }
      // else
      // {
      //   RCLCPP_INFO(get_logger(), "Unable to plan to target. Skipping to next");
      //   state_ = PositionerState::IDLE;
      // }
      break;
    }
    case PositionerState::EXECUTING:
    {
      enableCompensation = true;
      current_time = std::chrono::system_clock::now();
      elapsed_time = current_time - state_start_time;
      if (pose_at_target(current_pose)){
        start_time = std::chrono::system_clock::now();
        state_ = PositionerState::WAITING_AT_WEED;
        RCLCPP_INFO(get_logger(), "Arrived at weed, waiting for %fs", time_at_weed);
        pose_at_target(current_pose, 0);
        visualize_end_effector(current_pose);

        //double check with calculated weed position
        geometry_msgs::msg::PoseStamped test_pos;
        test_pos.pose.position.x = weed_position.at(0)+current_x_position-joint_states_[1];
        test_pos.pose.position.y = weed_position.at(1);
        test_pos.pose.position.z = weed_position.at(2);
        visualize_end_effector(test_pos, rviz_visual_tools::Colors::BLUE);
      }
      else if (elapsed_time.count() > TIMEOUT)
      {
        RCLCPP_INFO(get_logger(), "Problem moving to target. Skipping to next weed 123.");
        pose_at_target(current_pose, 0);
        //state_ = PositionerState::RECOVERY;
        RCLCPP_INFO(get_logger(), "Going back to Idle mode");
        state_ = PositionerState::IDLE;
      }
      break;
    }
    case PositionerState::WAITING_AT_WEED:
    { 
      enableCompensation = true;
      current_time = std::chrono::system_clock::now();
      elapsed_time = current_time - start_time;
      auto msg = std::make_unique<std_msgs::msg::String>();
      msg->data = WEED_TOUCH_MSG;
      touch_publisher_->publish(std::move(msg));
      if (elapsed_time.count() > time_at_weed){
        //state_ = PositionerState::MOVE_UP;
        n_weeds_destroyed++;
        RCLCPP_INFO(get_logger(), "Waiting done. %d weeds detected. %d weeds destroyed", n_weeds_detected, n_weeds_destroyed);
        //state_ = PositionerState::IDLE;
        if (check_weed_in_range())
        {
          state_ = PositionerState::PLAN;
        }else
        {
          state_ = PositionerState::MOVE_UP;
          //state_ = PositionerState::IDLE;
        }
      }
      break;
    }
    case PositionerState::MOVE_UP:
    { 
      target_pose = current_pose;
      target_pose.pose.position.z = Z_MOVING_UP;
      move_group_->setPositionTarget(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
      move_group_->setGoalTolerance(0.05);
      move_group_->asyncMove();
      state_ = PositionerState::MOVING_UP;
      break;
    }
    case PositionerState::MOVING_UP:
    {
      current_time = std::chrono::system_clock::now();
      elapsed_time = current_time - state_start_time;
      if(abs(current_pose.pose.position.z - target_pose.pose.position.z) < 0.05){
        if (actual_x_pos > MOVE_BACK_THRESHOLD)
        {
          state_ = PositionerState::MOVE_BACK;
        }else
        {
          state_ = PositionerState::MOVE_BACK;
        }
      }
      else if (elapsed_time.count() > TIMEOUT)
      {
        RCLCPP_INFO(get_logger(), "Problem moving up. Trying to recover.");
        pose_at_target(current_pose, 0);
        state_ = PositionerState::RECOVERY;
      }
      break;
    }
    case PositionerState::MOVE_BACK:
    {
      enableCompensation = false;
      target_pose = current_pose;  
      if ((abs(joint_states_[2])-0.01) > 0.05){
        target_pose.pose.position.x = 0.01; //joint_states_[2];
        move_group_->setNamedTarget("home");
        //move_group_->setPositionTarget(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
        move_group_->setGoalTolerance(0.05);
        target_pose = move_group_->getPoseTarget();
        move_group_->asyncMove();
      }
      else if (joint_states_[1] < 0.01)
      {
        state_ = PositionerState::HOMED;
      }
      state_ = PositionerState::MOVING_BACK;
      break;
    }
    case PositionerState::MOVING_BACK:
    {
      enableCompensation = false;
      current_time = std::chrono::system_clock::now();
      elapsed_time = current_time - state_start_time;
      auto jog_msg_zero = std::make_unique<std_msgs::msg::Float64MultiArray>();
      if (joint_states_[1] > 0.01)
      {
        jog_msg_zero->data.push_back(joint_states_[1] + back_vel*0.02); //TODO use variable for dt factor
        joint_pub_->publish(std::move(jog_msg_zero));
      }
      
      if (abs(current_pose.pose.position.x - target_pose.pose.position.x) < 0.05 && joint_states_[1] <= 0.01){
        state_ = PositionerState::HOMED;
      } 
      else if (elapsed_time.count() > TIMEOUT)
      {
        RCLCPP_INFO(get_logger(), "Problem moving back. Trying to recover.");
        pose_at_target(current_pose, 0);
        state_ = PositionerState::RECOVERY;
      }
      break;
    }
    case PositionerState::RECOVERY:
    {
      enableCompensation = false;
      for (auto &obj : active_collision_objects)
      {
        //clear planning scene
        planning_scene_interface_.removeCollisionObjects({obj.first});
      }
      active_collision_objects.clear();

      //clear current frame
      weeds_to_destroy.clear();
      crops_to_add.clear();

      //move up and then back
      state_ = PositionerState::MOVE_UP;
      break;
    }
  }

  //conveyor belt motion compensation
  if(enableCompensation){
    auto comp_pos = current_x_position-start_x;
    auto pos_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
    auto delta = comp_pos-prev_comp_pos;
    int sign = (delta < 0)? -1 : 1;
    if (abs(delta)>(-back_vel*0.02))
    {
      //rate limiting
      RCLCPP_INFO(get_logger(), "X_comp motion rate limited");
      comp_pos = prev_comp_pos + sign * -back_vel*0.02;
    }
    prev_comp_pos = comp_pos;
    pos_msg->data.push_back(comp_pos);
    joint_pub_->publish(std::move(pos_msg));
    /* if (state_ == PositionerState::PLAN || state_ == PositionerState::EXECUTING)
    {
        RCLCPP_INFO(get_logger(), "Compensation position x_comp= %4.3f", current_x_position-start_x);
    } */
  }

}

bool PositionerController::pose_at_target(geometry_msgs::msg::PoseStamped current_pose, double tolerance){
  float dX = current_pose.pose.position.x - target_pose.pose.position.x;
  float dY = current_pose.pose.position.y - target_pose.pose.position.y;
  float dZ = current_pose.pose.position.z - target_pose.pose.position.z;  
  //RCLCPP_INFO(get_logger(),"Pose<-->Target [%3.2f, %3.2f, %3.2f]", dX, dY, dZ);
  float distance = sqrt(dX*dX+dY*dY+dZ*dZ);
  auto dbg_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
  dbg_msg->data.push_back(distance);
  debug_pub_->publish(std::move(dbg_msg));
  if (tolerance==0)
  { //TMP for debug purposes
    RCLCPP_INFO(get_logger(), "Distance=%5.4f", distance);
    RCLCPP_INFO(get_logger(), "current=[%5.4f, %5.4f, %5.4f] ", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
    RCLCPP_INFO(get_logger(), "target=[%5.4f, %5.4f, %5.4f]", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
    RCLCPP_INFO(get_logger(), "Belt position=%5.4f", current_x_position);
  }
  return (distance < tolerance);
}

void PositionerController::visualize_end_effector(geometry_msgs::msg::PoseStamped pose, rviz_visual_tools::Colors color){
  pose.pose.position.x -= current_x_position;
  pose.pose.position.x += joint_states_[1];
  if (use_visual_tools_)
  {
    visual_tools_->publishCylinder(pose.pose, color);
    visual_tools_->trigger();
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto planner_controller = std::make_shared<PositionerController>();
  planner_controller->init();
  rclcpp::spin(planner_controller);
  rclcpp::shutdown();

  return 0;
}