#ifndef ALPHA_AUTOPILOT_H
#define ALPHA_AUTOPILOT_H
#include <alpha_autopilot/PID.h>
#include <alpha_autopilot/Types.h>
#include <alpha_msgs/FilteredState.h>
#include <alpha_msgs/RC.h>
#include <alpha_autopilot/AlphaMode.h>
#include <std_msgs/Empty.h>
#include <ros/ros.h>
#include <vector>
#include <alpha_drivers/PCA9685.h>

class AutoPilot{
 private:
  PID pid_roll;
  PID pid_pitch;
  PID pid_yaw;
  PID pid_z;
  PID pid_throttle;
  
  std::vector<int> trim;
  std::vector<int> rc_in;
  
  ros::Subscriber rc_sub;
  ros::NodeHandle nh;
  ros::Publisher calibrate_pub;
  ros::Publisher rcout_pub;
  ros::Subscriber state_sub;
  ros::Subscriber marker_state_sub;

  int drop_state;
  
  AlphaState imu_state;
  AlphaState marker_state;

  Ada_ServoDriver pwm;

 public:
  AutoPilot();
  ~AutoPilot();  
  void init();
  AlphaMode* current_mode;
  void update();
  void setRCIn(std::vector<int> &rc_in);
 private:

  void setRCOut(std::vector<int> &rc_out);
  void stateCB(alpha_msgs::FilteredState::ConstPtr msg);
  void marker_stateCB(alpha_msgs::FilteredState::ConstPtr msg);
  std::vector<int> compute_auto_rc_out(float roll_effort, float pitch_effort, float throttle);
  std::vector<int> compute_manual_rc_out(std::vector<int> rc_in);
  void send_calibrate_request();

};
#endif //ALPHA_AUTOPILOT_H
