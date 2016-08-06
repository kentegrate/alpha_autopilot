#include <ros/ros.h>
#include <alpha_msgs/DynamicPIDConfig.h>
#include <dynamic_reconfigure/server.h>
class PID{
 public:
  void set_setpoint(float _setpoint);
  float update(float _state);
  PID(std::string _plant_name);
  void initialize();
 private:
  void _reconfig_CB(alpha_autopilot::DynamicPIDConfig &config, uint32_t level);

  float setpoint;
  ros::Time prev_time;

  float error_integral;

  float Kp, Ki, Kd;   // PID loop parameters

  // Cutoff frequency for the derivative calculation in Hz.
  // Negative -> Has not been set by the user yet, so use a default.
  float cutoff_frequency; 

  // Used in filter calculations. Default 1.0 corresponds to a cutoff frequency at
  // 1/4 of the sample rate.
  float c;

  // Used to check for tan(0)==>NaN in the filter calculation
  float tan_filt;

  // Upper and lower saturation limits
  float upper_limit;
  float lower_limit;
  float windup_limit; // Anti-windup term. Limits the absolute value of the integral term.
  std::vector<float> error;
  std::vector<float> filtered_error;
  std::vector<float> error_deriv;
  std::vector<float> filtered_error_deriv;
  int loop_counter;
  std::string plant_name;
  ros::NodeHandle private_nh;
  dynamic_reconfigure::Server<alpha_autopilot::DynamicPIDConfig> cfg_server;
  dynamic_reconfigure::Server<alpha_autopilot::DynamicPIDConfig>::CallbackType f;
  bool first_reconfig;
  

};
