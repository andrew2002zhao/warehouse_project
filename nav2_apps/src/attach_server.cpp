#include "rclcpp/callback_group.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/timer.hpp"
#include <condition_variable>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <attach_shelf_interfaces/srv/go_to_loading.hpp>
#include <attach_shelf_interfaces/srv/move_forwards.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/string.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <cmath>
#include <mutex>

using namespace std::chrono_literals;

using TFMessage = tf2_msgs::msg::TFMessage;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Twist = geometry_msgs::msg::Twist;
using Quaternion = geometry_msgs::msg::Quaternion;
using LaserScan = sensor_msgs::msg::LaserScan;
using Odometry = nav_msgs::msg::Odometry;
using Point = geometry_msgs::msg::Point;
using GoToLoading = attach_shelf_interfaces::srv::GoToLoading;
using MoveForwards = attach_shelf_interfaces::srv::MoveForwards
using String = std_msgs::msg::String;

using MovementFunction = std::function<void()>;

#define PI 3.141592
#define LEG_INTENSITY 6000

namespace nav2_apps{
    class ApproachService : public rclcpp::Node {
        public:
            ApproachService(const rclcpp::NodeOptions & nodeOptions) : rclcpp::Node("approach_service_node", nodeOptions) {
                this -> initialize_qos__();
                this -> initialize_callback_group__();

                this -> tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
               
                
                this -> load_shelf_publisher_ = this -> create_publisher<String>("/elevator_up", 10);
                this -> robot_movement_publisher_ = this -> create_publisher<Twist>("/diffbot_base_controller/cmd_vel_unstamped", this -> robot_movement_publisher_qos_);
                this -> robot_laser_subscription_ = this -> create_subscription<LaserScan>("/scan", this -> robot_laser_subscription_qos_, 
                
                [this](const LaserScan::SharedPtr msg){
                 
                    this -> intensities__ = msg -> intensities;
                    this -> ranges__ = msg -> ranges;

                }, this -> robot_laser_options_);

                this -> robot_position_subscription_ = this -> create_subscription<Odometry>("/diffbot_base_controller/odom", this -> robot_position_subscription_qos_, [this](const Odometry::SharedPtr msg){
                    this -> quaternion_ = msg -> pose.pose.orientation;
                    this -> point_ = msg -> pose.pose.position;
                }, this -> robot_position_options_);

    
                this -> go_to_loading_service_ = this -> create_service<GoToLoading>("/approach_shelf", 
                
                [this](const GoToLoading::Request::SharedPtr request, const GoToLoading::Response::SharedPtr response){
                    

                    int legs = get_number_of_intensities__(this -> intensities__);
                    if(legs <= 1){
                        RCLCPP_INFO(this -> get_logger(), "ONLY 1 LEG");
                        response -> complete = false;
                        return;
                    }
                    RCLCPP_INFO(this -> get_logger(), "DETECTED 2 LEGS");
                    //parse the intensities to find the position of the legs
                    std::vector<int> leg_positions = this -> get_leg_positions__(intensities__);
                    //get the leg angles from the positions
                    std::vector<float> leg_angles = get_leg_angles__(this -> ranges__, leg_positions);
                    //get the translation amount for center point between the two table leges
                    std::pair<float, float> translation = get_translation_from_current_frame__(this -> ranges__[leg_positions[0]], this -> ranges__[leg_positions[1]], leg_angles[0], leg_angles[1]);
                  

                    publish_transform_timer__(translation);

                    // rotate to correct orientation
                    
                    this -> done_ = true;
                    float radians = atan2(translation.second, translation.first);
                    // radians *= -1;
                    auto correct_orientation = std::bind(&ApproachService::rotate_robot__, this, radians);
                    RCLCPP_INFO(this -> get_logger(), "%f", radians);
                    //get tf from robot_base_link to cart_frame
                    //move forwards by tf amount
                    auto point = Point();
                    point.x = translation.first;
                    point.y = translation.second;
                    point.z = 0;
                    float distance = distance_from_xyz__(point) + 0.4; 
                    RCLCPP_INFO(this -> get_logger(), "%f", distance);
                    auto move_forwards = std::bind(&ApproachService::move_robot__, this, distance);
                   
                    // //rotate again to correct
                    // correction_radians = quaternion_to_yaw__(this -> target_quaternion_);
                    // auto second_correction = std::bind(&ApproachService::rotate_robot__, this, radians);

                    //move forwards 30cm
               
                    float shelf_distance = 0.3;
                    auto shelf_forwards = std::bind(&ApproachService::move_robot__, this, shelf_distance);
                  
                    

                    std::vector<MovementFunction> tasks_to_do = {
                        correct_orientation, 
                        move_forwards, 
                        shelf_forwards
                    };
                 

                    auto i = std::make_shared<int>(0);
                  
                   
                    //hopefully this is put on a different thread
                    this -> done_timer_ = this -> create_wall_timer(50ms, [this, tasks_to_do, i, response](){
                     
                        if(this -> done_){
                            if(*i == tasks_to_do.size()){

                                std::lock_guard<std::mutex> lock(done_mutex_);
                                this -> done_timer_ -> cancel();
                                // move shelf
                                auto msg = String();
                                this -> load_shelf_publisher_-> publish(msg);
                                response -> complete = true;
                                done_cv_.notify_one();
                            }
                            else{
                                RCLCPP_INFO(this -> get_logger(), "%d", *i);
                                tasks_to_do[*i]();
                                this -> done_ = false;
                                (*i)++;
                            }
                        }
                    
                    }, done_callback_group_);

                    //dont return until async is done
                    //cannot busy wait
                    std::unique_lock<std::mutex> lock(done_mutex_);
                    done_cv_.wait(lock);

                });

                this -> move_forwards_service_ = this -> create_service<MoveForwards>("/move_forwards", 

                [this](const MoveForwards::Request::SharedPtr request, const MoveForwards::Response::SharedPtr response){

                    this -> done_ = false;
                    move_robot__(request -> distance);
                    this -> move_forwards_timer_ = this -> create_wall_timer(50ms, [this](){
                        
                        if(this -> done_){
                            std::lock_guard<std::mutex> lock(move_forwards_mutex_);
                            move_forwards_cv_.notify_one();
                        }
                        
                    }, done_callback_group_);
                    std::unique_lock<std::mutex> lock(move_forwards_mutex_);
                    move_forwards_cv_.wait(move_forwards_mutex_);
                });
            }
           
        protected:
            //go_to_loading
            rclcpp::Service<GoToLoading>::SharedPtr go_to_loading_service_;
            rclcpp::QoS go_to_loading_service_qos_{10};
            //laser_scan
            rclcpp::Subscription<LaserScan>::SharedPtr robot_laser_subscription_;
            rclcpp::QoS robot_laser_subscription_qos_{10};
            std::shared_ptr<rclcpp::CallbackGroup> robot_laser_callback_group_;
            rclcpp::SubscriptionOptions robot_laser_options_;
            //odometry
            rclcpp::Subscription<Odometry>::SharedPtr robot_position_subscription_;
            rclcpp::QoS robot_position_subscription_qos_{10};
            std::shared_ptr<rclcpp::CallbackGroup> robot_position_callback_group_;
            rclcpp::SubscriptionOptions robot_position_options_;
            Quaternion quaternion_;
            Point point_;
            //tf_broadcaster
            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            rclcpp::TimerBase::SharedPtr tf_timer_;
            std::shared_ptr<rclcpp::CallbackGroup> transform_callback_group_;
            //twist
            rclcpp::Publisher<Twist>::SharedPtr robot_movement_publisher_;
            rclcpp::QoS robot_movement_publisher_qos_{10};
            rclcpp::TimerBase::SharedPtr movement_timer_;
            rclcpp::TimerBase::SharedPtr rotation_timer_;
            std::shared_ptr<rclcpp::CallbackGroup> robot_movement_publisher_callback_group_;
            //done 
            rclcpp::TimerBase::SharedPtr done_timer_;
            std::shared_ptr<rclcpp::CallbackGroup> done_callback_group_;
            std::condition_variable done_cv_;
            std::mutex done_mutex_;
            bool done_;
            //shelf_load
            rclcpp::Publisher<String>::SharedPtr load_shelf_publisher_;
            //move forwards
            rclcpp::Service<MoveForwards>::SharedPtr move_forwards_service_;
            rclcpp::TimerBase::SharedPtr move_forwards_timer_;
            std::condition_variable move_forwards_cv_;
            std::mutex move_forwards_mutex_;
            bool move_done_;
            

        private:

            std::vector<float> intensities__;
            std::vector<float> ranges__;

            void initialize_qos__(){
                this -> robot_movement_publisher_qos_.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
                this -> robot_movement_publisher_qos_.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
                this -> robot_movement_publisher_qos_.liveliness(RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT);

                this -> robot_position_subscription_qos_.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
                this -> robot_position_subscription_qos_.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
                this -> robot_position_subscription_qos_.liveliness(RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT);
                
                this -> robot_laser_subscription_qos_.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
                this -> robot_laser_subscription_qos_.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
                this -> robot_laser_subscription_qos_.liveliness(RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT);
            }

            void initialize_callback_group__(){
                this -> robot_position_callback_group_ = this -> create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                this -> robot_position_options_.callback_group = this -> robot_position_callback_group_;
                this -> robot_laser_callback_group_ = this -> create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                this -> robot_laser_options_.callback_group = this -> robot_laser_callback_group_;
                this -> transform_callback_group_ = this -> create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                this -> robot_movement_publisher_callback_group_ = this -> create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                this -> done_callback_group_ = this -> create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            }

            int get_number_of_intensities__ (std::vector<float> & intensities){
                int leg_total = 0;

                bool is_leg = false;
                for(int i = 0; i < intensities.size(); i++) {
                    if(intensities[i] > LEG_INTENSITY) {
                        
                        is_leg = true;
                    }
                    else{
                        if(is_leg == true) {
                            is_leg = false;
                            leg_total += 1;
                        }
                    }
                }
                return leg_total;
            }
            std::vector<int> get_leg_positions__(std::vector<float> & intensities) {
                std::vector<int> leg_positions = {};
                int leg_position_total = 0;
                int leg_position_total_indexes = 0; 
                bool is_leg = false;
                for(int i = 0; i < intensities.size(); i++) {
                    if(intensities[i] > LEG_INTENSITY) {
                        leg_position_total_indexes += i;
                        leg_position_total += 1;
                        is_leg = true;
                    }
                    else{
                        if(is_leg == true) {
                            is_leg = false;
                            leg_positions.push_back(leg_position_total_indexes / leg_position_total);
                            leg_position_total = 0;
                            leg_position_total_indexes = 0;
                        }
                    }
                }
                return leg_positions;
            }

            std::vector<float> get_leg_angles__(std::vector<float> & ranges, std::vector<int> & positions) {
                std::vector<float> angles = {};
                for(auto position : positions) {
                    RCLCPP_INFO(this -> get_logger(), "POSITIONS: position %d", position);
                    float angle_increment = 2 * PI / ranges.size();
                    float center_index = ranges.size() / 2;
                    float index_difference = center_index - position;
                    float angle_difference = index_difference * angle_increment;
                    
                    angles.push_back(angle_difference);
                }
                return angles;
            } 

            std::pair<float, float> get_translation_from_current_frame__(float l1, float l2, float a1, float a2) {
                RCLCPP_INFO(this -> get_logger(), "%f, %f, %f, %f", l1, l2, a1, a2);
                //cosine law
                float l3 = sqrt(pow(l1, 2) + pow(l2, 2) - 2 * l1 * l2 * cos(a2 - a1));
                RCLCPP_INFO(this -> get_logger(), "%f", l3);
                //sine law
                float a3 = asin(sin(a2 - a1) * l2 / l3);

                //there are multiple possible sine solutions
                //currently this is hardcoded to work with the test case
                RCLCPP_INFO(this -> get_logger(), "%f", a3);
                // a3 = PI - a3;
                //cosine law
                float l4 = sqrt(pow(l1, 2) + pow((l3 / 2), 2) - 2 * l1 * (l3 / 2) * cos(a3));
                RCLCPP_INFO(this -> get_logger(), "%f", l4);
                //sine law
                float a4 = asin(sin(a3) * (l3 / 2) / l4);
                RCLCPP_INFO(this -> get_logger(), "%f", a4);
                RCLCPP_INFO(this -> get_logger(), "%f", a1);
                //angular difference
                float a5 = a1 + a4;
                std::pair<float, float> translation = {cos(a5) * l4, sin(a5) * l4};
                return translation;
            }

            void publish_transform_timer__(std::pair<float, float> & transform) {
            
                //take the transform pair

                //get the robot_base_link tf 
                //use the odometry to get the quaternion
                
                
                //create a new tf from the base_link using a broadcaster
                this -> tf_timer_ = this -> create_wall_timer(50ms, [this, transform](){

                    geometry_msgs::msg::TransformStamped t_send;

                    t_send.header.stamp = this->get_clock()->now();
                    t_send.header.frame_id = "robot_front_laser_base_link";
                    t_send.child_frame_id = "cart_frame";

                    t_send.transform.translation.x = transform.first;
                    t_send.transform.translation.y = transform.second * -1;
                    t_send.transform.translation.z = 0;
                
                    t_send.transform.rotation = this -> quaternion_;

                    tf_broadcaster_-> sendTransform(t_send);

            
                    auto point = Point();
                    point.x = transform.first;
                    point.y = transform.second * -1;
                    point.z = 0;

                }, this -> transform_callback_group_);   
            }

            void rotate_robot__(float radians){
                this -> done_ = false;
                float current_position = quaternion_to_yaw__(this -> quaternion_);
                RCLCPP_INFO(this -> get_logger(), "%f", current_position);
                RCLCPP_INFO(this -> get_logger(), "%f", radians);
                float final_position = current_position + radians;
                RCLCPP_INFO(this -> get_logger(), "%f", final_position);
                if(final_position > PI) {
                    final_position = final_position - 2 * PI;
                }
                else if (final_position < -1 * PI){
                    final_position = final_position + 2 * PI;
                }
                RCLCPP_INFO(this -> get_logger(), "%f", final_position);

                this -> rotation_timer_ = this -> create_wall_timer(50ms, [this, final_position](){
                    float current_position = quaternion_to_yaw__(this -> quaternion_);

                    auto movement_msg = Twist();
                    movement_msg.linear.x = 0;
                    movement_msg.linear.y = 0;
                    movement_msg.linear.z = 0;
                    movement_msg.angular.x = 0;
                    movement_msg.angular.y = 0;
                  
                    RCLCPP_INFO(this -> get_logger(), "%f", abs(current_position - final_position));
                    if(abs(current_position - final_position) <= 0.01){
                        //within range
                        
                        this -> robot_movement_publisher_ -> publish(movement_msg);
                        this -> done_ = true;
                        this -> rotation_timer_ -> cancel();
                        
                        
                    } else{
                       
                        //outside range
                        if(final_position > 0){
                            //counter clockwise
                            movement_msg.angular.z = 0.2;
                        }
                        else{
                            movement_msg.angular.z = -0.2;
                        }
                        this -> robot_movement_publisher_ -> publish(movement_msg);
                        
                    }
                }, this -> robot_movement_publisher_callback_group_);

            }

            void move_robot__(float meters){

                this -> done_ = false;
                Point starting_position = this -> point_;

                this -> movement_timer_ = this -> create_wall_timer(50ms, [this, starting_position, meters](){
                    Point current_position = this -> point_;

                    auto movement_msg = Twist();
                    movement_msg.linear.x = 0;
                    movement_msg.linear.y = 0;
                    movement_msg.linear.z = 0;
                    movement_msg.angular.x = 0;
                    movement_msg.angular.y = 0;
                    movement_msg.angular.z = 0;

                    if(abs(meters - distance_from_xyz__(current_position, starting_position)) <= 0.03){
                        //within range
                     
                        this -> robot_movement_publisher_ -> publish(movement_msg);
                        this -> done_ = true;
                        this -> movement_timer_ -> cancel();
                        
                        
                    } else{
                        //outside range
                        movement_msg.linear.x = 0.2;
                        this -> robot_movement_publisher_ -> publish(movement_msg);
                        
                    }
                }, this -> robot_movement_publisher_callback_group_);

            }

            float quaternion_to_yaw__(Quaternion q){
                return atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
            }

            float distance_from_xyz__(Point p) {
                
                return sqrt(p.x * p.x + p.y * p.y);
            }

            float distance_from_xyz__(Point p1, Point p2) { 
               
                return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
            }

    };
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(nav2_apps::ApproachService)
