#ifndef STEERING_CONTROLLER__HPP
#define STEERING_CONTROLLER__HPP
#include "ros/ros.h"
#include <std_msgs/String.h>
#include <string>

#define CNTRL_MD_RUN 0
#define CNTRL_MD_ROT 1
#define CNTRL_MD_TRS 2

#define DEFAULT_NODE_NAME "steering_controller"
#define DEFAULT_TOPIC_NAME "sub_test"

namespace steering_controller
{
struct can_data
{
  int id;
  uint32_t data;
};

class SteeringController
{
 public:
  SteeringController();

  // Control
  void control_callback();
  void run();
  void rotate();
  void transform();

  // Subscribe commanded through topic
  void sub_command_callback(const std_msgs::String::ConstPtr& msg);

  // Send command to arduino by serial com
  void send_command_to_arduino();

  // Get encoder value from arduino
  void get_state_from_arduino();

//  private:
  // // Hardware parameters
  // Assuming all wheel have the same radius
  double wheel_radius_;

  // Wheel distance between left and right 
  // Assuming same distance for each pair 
  double wheel_distance_width_;

  // Wheel distance between front and rear 
  // Assuming same distance for each pair
  double wheel_distance_length_;

  // Radius when rover is turing 
  // Calculated from wheel radius and distance
  double rover_turning_radius_ ;
  
  // Timeout to consider cmd_vel commands old
  double cmd_vel_timeout_;

  // Update rate for main control
  double update_rate_;

  // 
  int control_mode_;

  can_data cmd_tire_LF;
  can_data cmd_tire_LB;
  can_data cmd_tire_RF;
  can_data cmd_tire_RB;
  can_data cmd_steer_LF;
  can_data cmd_steer_LB;
  can_data cmd_steer_RF;
  can_data cmd_steer_RB;

  double des_rover_vel_;
  double des_rover_ang_vel_;

  // Ros node to subscribe topic
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Timer timer_;

  // Get parameter and store into method from yaml
  // Then calculate other params 
  void setup_param();

}; // end of class : SteeringController
} // end of namespace : steering_controller
#endif