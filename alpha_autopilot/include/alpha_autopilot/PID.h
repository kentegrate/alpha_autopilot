#include <ros/ros.h>
#include <alpha_autopilot/DynamicPIDConfig.h>
#include <dynamic_reconfigure/server.h>
class PID{
  void set_setpoint(double _setpoint);
  double update(double _state);
  PID(std::string _plant_name);
 private:
  void _reconfig_CB(alpha_autopilot::DynamicPIDConfig &config, uint32_t level);

  double setpoint;
  ros::Time prev_time;
  ros::Duration delta_t;

  double error_integral;

  double Kp, Ki, Kd;   // PID loop parameters

  // Cutoff frequency for the derivative calculation in Hz.
  // Negative -> Has not been set by the user yet, so use a default.
  double cutoff_frequency; 

  // Used in filter calculations. Default 1.0 corresponds to a cutoff frequency at
  // 1/4 of the sample rate.
  double c;

  // Used to check for tan(0)==>NaN in the filter calculation
  double tan_filt;

  // Upper and lower saturation limits
  double upper_limit;
  double lower_limit;
  double windup_limit; // Anti-windup term. Limits the absolute value of the integral term.
  std::vector<double> error;
  std::vector<double> filtered_error;
  std::vector<double> error_deriv;
  std::vector<double> filtered_error_deriv;
  int loop_counter;
  std::string plant_name;

  dynamic_reconfigure::Server<alpha_autopilot::DynamicPIDConfig> cfg_server;
  dynamic_reconfigure::Server<alpha_autopilot::DynamicPIDConfig>::CallbackType f;
  bool first_reconfig;
  
  ros::NodeHandle private_nh;
};
