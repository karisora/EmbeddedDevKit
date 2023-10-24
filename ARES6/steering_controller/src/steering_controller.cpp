#include "steering_controller.hpp"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <sstream>
#include <math.h>

// TODO
// :Handle node and topic name in better way
// :Setup timeout in case of communication error
// :Set real topic 
// :Write serial com code
// TODO

namespace steering_controller
{
  SteeringController :: SteeringController(){
    // // Constructor 
    setup_param();

    // Setup ros subscriber 
    sub_ = nh_.subscribe<std_msgs::String>(
      DEFAULT_TOPIC_NAME, 10, boost::bind(&SteeringController::sub_command_callback, this, _1));

    // Setup ros ros timer
    // Connected to main control
    double duration = 1.0 / update_rate_;
    timer_ = nh_.createTimer(ros::Duration(duration), boost::bind(&SteeringController::control_callback, this));

    // Initial control mode set to run
    control_mode_ = CNTRL_MD_RUN;

    // Setup CAN data
    cmd_tire_RF.id = 0x200;
    cmd_tire_RB.id = 0x201;
    cmd_tire_LF.id = 0x202;
    cmd_tire_LB.id = 0x203;
    cmd_steer_RF.id = 0x210;
    cmd_steer_RB.id = 0x211;
    cmd_steer_LF.id = 0x212;
    cmd_steer_LB.id = 0x213;
  } // SteeringController

  void SteeringController :: setup_param(){
    // Get ros node namespace 
    ros::NodeHandle pnh("~");

    // Store parameters
    pnh.getParam("steering_controller/wheel_radius", wheel_radius_);
    pnh.getParam("steering_controller/wheel_distance_width", wheel_distance_width_);
    pnh.getParam("steering_controller/wheel_distance_length", wheel_distance_length_);
    pnh.getParam("steering_controller/update_rate", update_rate_);

    // TODO: error handling if failed to read param

    // Calculate other parameters
    double len_dig = pow(wheel_distance_width_, 2) + pow(wheel_distance_length_, 2);
    double turn_diam = std::sqrt(len_dig);
    rover_turning_radius_ = turn_diam * 0.5;

    // Show them up to confirm 
    ROS_INFO("wheel_radius is %.3lf m", wheel_radius_);
    ROS_INFO("wheel_distance_width is %.3lf m", wheel_distance_width_);
    ROS_INFO("wheel_distance_length is %.3lf m", wheel_distance_length_);
    ROS_INFO("rover_turning_radius is %.3lf m", rover_turning_radius_);
    ROS_INFO("update_rate is %ld Hz", update_rate_);
  } // setup_param

  // Main controller
  // Frequently called by ros timer
  void SteeringController :: control_callback(){
    // ROS_INFO("control loop");
    switch (control_mode_)
    {
    case CNTRL_MD_RUN:
      run();
      ROS_INFO("cmd is %d", cmd_tire_RF.id);
      break;

    case CNTRL_MD_ROT:
      rotate();
      break;

    case CNTRL_MD_TRS: 
      transform();
      break;
    
    default:
      break;
    }
    return;
  } // control_callback
  
  void SteeringController :: run(){
    double whl_rot_v = des_rover_vel_ / wheel_radius_; // m/s -> rad/s

    // Convert into CAN command
    cmd_tire_LF.data = 256;
    cmd_tire_LB.data = 256;
    cmd_tire_RF.data = 256;
    cmd_tire_RB.data = 256;
    cmd_steer_LF.data = 256;
    cmd_steer_LB.data = 256;
    cmd_steer_RF.data = 256;
    cmd_steer_RB.data = 256;
  } // run

  void SteeringController :: rotate(){
    double whl_rot_v = des_rover_ang_vel_ * rover_turning_radius_ / wheel_radius_; // rad/s -> rad/s

    // Convert into CAN command
    cmd_tire_LF.data = 256;
    cmd_tire_LB.data = 256;
    cmd_tire_RF.data = 256;
    cmd_tire_RB.data = 256;
    cmd_steer_LF.data = 256;
    cmd_steer_LB.data = 256;
    cmd_steer_RF.data = 256;
    cmd_steer_RB.data = 256;
  } // rotate

  void SteeringController :: transform(){

    // Convert into CAN command
    cmd_tire_LF.data = 256;
    cmd_tire_LB.data = 256;
    cmd_tire_RF.data = 256;
    cmd_tire_RB.data = 256;
    cmd_steer_LF.data = 256;
    cmd_steer_LB.data = 256;
    cmd_steer_RF.data = 256;
    cmd_steer_RB.data = 256;
  } // transform 

  void SteeringController :: send_command_to_arduino(){

  } // send_command_to_arduino

  void SteeringController :: get_state_from_arduino(){

  } // get_state_from_arduino

  void SteeringController :: sub_command_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("Subscribe topic");
  } // sub_command_callback
} // end of namespace: steering_controller


int main(int argc, char **argv)
{
  ros::init(argc, argv, DEFAULT_NODE_NAME);
  ros::NodeHandle nh;

  steering_controller :: SteeringController st_controller;    

  ros::spin(); // "control_callback" and "sub_command_callback" 

  return 0;
}