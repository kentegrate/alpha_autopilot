#ifndef ALPHA_AUTOPILOT_H
#define ALPHA_AUTOPILOT_H
#include <alpha_autopilot/PID.h>
#include <alpha_autopilot/Types.h>
#include <alpha_msgs/FilteredState.h>
#include <alpha_msgs/RC.h>
#include <alpha_autopilot/AlphaMode.h>
#include <ros/ros.h>
#include <vector>
class AutoPilot{
 private:
  PID pid_roll;
  PID pid_pitch;
  
  std::vector<int> trim;
  std::vector<int> rc_in;
  
  ros::Subscriber rc_sub;
  ros::NodeHandle nh;
  ros::Publisher rc_pub;
  ros::Subscriber state_sub;
  
  AlphaState state;

 public:
  AutoPilot();
  ~AutoPilot();  
  void init();
  AlphaMode* current_mode;
  void update();
 private:
  void rcInputCB(alpha_msgs::RC::ConstPtr msg);
  void publishRC(std::vector<int> &rc_out);
  void stateCB(alpha_msgs::FilteredState::ConstPtr msg);
  std::vector<int> compute_auto_rc_out(double roll_effort, double pitch_effort, double throttle);
  std::vector<int> compute_manual_rc_out(std::vector<int> rc_in);

};
#endif //ALPHA_AUTOPILOT_H
