// Includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <rviz_visual_tools/rviz_visual_tools.hpp>

#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <ctime>
#include <chrono>
#include <thread>
#include <map>
#include <queue>

#include <iostream>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include "detection_interfaces/msg/detections.hpp"
#include "detection_interfaces/msg/detected.hpp"
#include "detection_interfaces/msg/conveyor.hpp"

using std::placeholders::_1;

class PlantTracking : public rclcpp::Node
{
    public:
        PlantTracking() : Node("plant_tracking"){
            visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(world_frame,"/rviz_visual_markers", this));
            visual_tools_->enableBatchPublishing();
            visual_tools_->enableFrameLocking();
            visual_tools_->trigger();

            detections_sub = this->create_subscription<detection_interfaces::msg::Detections>(
                "/detections", ROS_QUEUE_SIZE, std::bind(&PlantTracking::detections_callback, this, _1));
            conveyor_sub = this->create_subscription<detection_interfaces::msg::Conveyor>(
                "/conveyor", ROS_QUEUE_SIZE, std::bind(&PlantTracking::conveyor_callback, this, _1));
            
            detections_pub = this->create_publisher<detection_interfaces::msg::Detections>("/filtered_detections", ROS_QUEUE_SIZE);
        }
        
    private:
        // Functions
        void conveyor_callback(const detection_interfaces::msg::Conveyor::SharedPtr conveyor_msg) {
            double offset = 0;
            std::vector<double> position;
            double x_pos;
            double robot_x_length = 1.0;
            double threshold_x_pos = -0.1;
            detection_interfaces::msg::Detections msg;
            detection_interfaces::msg::Detected detection;
            bool crops_in_range, weeds_in_range;
            int n_detections_to_send;
            
            comp_vel = conveyor_msg->speed;
            current_x_position = conveyor_msg->distance_travelled;

            //check for crops/weeds to send
            //send crops from received detections when in planning region
            crops_in_range = !crops_to_add.empty();
            n_detections_to_send = 0;
            msg.detected.clear();
            msg.header.stamp = this->get_clock()->now();
            RCLCPP_DEBUG(this->get_logger(), "Current position: x=%4.3f ", current_x_position);
            std::lock_guard<std::mutex> lck_crops(m_crops);
            while(crops_in_range)
            {
                position = crops_to_add.front();
                x_pos = position.at(0)+current_x_position;
                
                RCLCPP_DEBUG(this->get_logger(), "First of %d crops at x=%4.3f ", crops_to_add.size(), x_pos);
                if (x_pos > threshold_x_pos && x_pos < robot_x_length)
                {
                    detection.type = "crop";
                    detection.position.x = position.at(0);
                    detection.position.y = position.at(1);
                    detection.position.z = position.at(2);
                    msg.detected.push_back(detection);
                    crops_to_add.pop();
                    n_detections_to_send++;
                    crops_in_range = !crops_to_add.empty();
                }
                else
                {
                    crops_in_range = false;
                }
            }

            weeds_in_range = !weeds_to_destroy.empty();

            std::lock_guard<std::mutex> lck_weeds(m_weeds);
            while(weeds_in_range)
            {
                position = weeds_to_destroy.front();
                x_pos = position.at(0)+current_x_position;
                RCLCPP_DEBUG(this->get_logger(), "First of %d weeds at x=%4.3f ", weeds_to_destroy.size(), x_pos);
                if (x_pos > threshold_x_pos && x_pos < robot_x_length)
                {
                    detection.type = "weed";
                    detection.position.x = position.at(0);
                    detection.position.y = position.at(1);
                    detection.position.z = position.at(2);
                    msg.detected.push_back(detection);
                    weeds_to_destroy.pop();
                    n_detections_to_send++;
                    weeds_in_range = !weeds_to_destroy.empty();
                }
                else
                {
                    weeds_in_range = false;
                }
            }

            if (n_detections_to_send>0)
            {
                detections_pub->publish(msg);
            }
        }

        void detections_callback(const detection_interfaces::msg::Detections::SharedPtr msg){

            int n_crops=0, n_weeds=0;

            std::lock_guard<std::mutex> lck_crops(m_crops);
            std::lock_guard<std::mutex> lck_weeds(m_weeds);
            for(auto detection : msg->detected){
                
                if(detection.type == "crop"){
                    std::vector<double> location{detection.position.x, detection.position.y, detection.position.z};
                    //visual_tools_->publishSphere(detection.position, rviz_visual_tools::Colors::GREEN, rviz_visual_tools::Scales::LARGE, "sphere_crop", ++sphere_id);
                    crops_to_add.push(location);
                    n_crops++;
                }

                if(detection.type == "weed"){
                    std::vector<double> location{detection.position.x, detection.position.y, detection.position.z};
                    if (addWeedOnlyIfNew(location))
                    {
                        //check for already detected weeds in list
                        //visual_tools_->publishSphere(detection.position, rviz_visual_tools::Colors::YELLOW, rviz_visual_tools::Scales::LARGE, "sphere_weed", ++sphere_id);
                        n_weeds++;
                    }
                }
            }
            if (n_crops+n_weeds > 0)
            {
                //visual_tools_->trigger();
                RCLCPP_INFO(this->get_logger(), "Detections received: %d crops and %d weeds", n_crops, n_weeds);
            }
        }

        bool addWeedOnlyIfNew(std::vector<double> item){
            bool addWeed = false;
            if (weeds_to_destroy.empty()){
                addWeed = true;
            }
            else{
                auto last = weeds_to_destroy.back();
                float dX = last.at(0) - item.at(0);
                float dY = last.at(1) - item.at(1);
                float distance = sqrt(dX*dX+dY*dY);
                addWeed = (distance > SAME_WEED_DISTANCE);
            }
            if (addWeed){
                weeds_to_destroy.push(item);
                return true;
            }
            return false;
        }

        // Variables
        std::string world_frame = "world";
        std::vector<std::string> crop_ids;
        int crop_count = 0;
        double current_x_position;

        const size_t ROS_QUEUE_SIZE = 10;
        const double SAME_WEED_DISTANCE = 0.10;


        float comp_vel = 0; // Compensation velocity
        int sphere_id = 0;

        std::mutex m_weeds;
        std::mutex m_crops;
        std::mutex m_objects;
        std::queue<std::vector<double>> weeds_to_destroy;
        std::queue<std::vector<double>> crops_to_add;
        std::map<std::string, double> active_collision_objects; //maps x-coordinate to id to be able to remove crop when it is passed
        rclcpp::Subscription<detection_interfaces::msg::Conveyor>::SharedPtr conveyor_sub;
        rclcpp::Subscription<detection_interfaces::msg::Detections>::SharedPtr detections_sub;
        rclcpp::Publisher<detection_interfaces::msg::Detections>::SharedPtr detections_pub;
        
        // For visualizing things in rviz
        rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlantTracking>());
  rclcpp::shutdown();
  return 0;

}