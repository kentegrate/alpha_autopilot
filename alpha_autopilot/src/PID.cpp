#include <alpha_autopilot/PID.h>

void PID::initialize(){
  setpoint = 0;
  error_integral = 0.;

  // Cutoff frequency for the derivative calculation in Hz.
  // Negative -> Has not been set by the user yet, so use a default.
  cutoff_frequency = -1; 

  // Used in filter calculations. Default 1.0 corresponds to a cutoff frequency at
  // 1/4 of the sample rate.
  c=1.;

  // Used to check for tan(0)==>NaN in the filter calculation
  tan_filt = 1.;

  // Upper and lower saturation limits

  loop_counter = 0;
  prev_time = ros::Time();

  for(int i = 0; i < 3; i++){
    error[i] = 0;
    filtered_error[i] = 0;
    error_deriv[i] = 0;
    filtered_error_deriv[i]=0;
  }
}

  

PID::PID(std::string _plant_name):error(3,0), filtered_error(3,0), error_deriv(3,0), filtered_error_deriv(3,0),private_nh("~/"+_plant_name),cfg_server(private_nh){


  plant_name = _plant_name;

  f = boost::bind(&PID::_reconfig_CB,this,_1,_2);
  cfg_server.setCallback(f);
  first_reconfig = false;

  private_nh.getParam("Kp",Kp);
  private_nh.getParam("Ki",Ki);
  private_nh.getParam("Kd",Kd);

  private_nh.getParam("upper_limit",upper_limit);
  private_nh.getParam("lower_limit",lower_limit);
  private_nh.getParam("windup_limit",windup_limit);

  private_nh.getParam("cutoff_frequency",cutoff_frequency);
  
}



double PID::update(double _state){

  if ( !((Kp<=0. && Ki<=0. && Kd<=0.) || (Kp>=0. && Ki>=0. && Kd>=0.)) ) // All 3 gains should have the same sign
    {
      ROS_WARN("All three gains (Kp, Ki, Kd) should have the same sign for stability.");
    }
  error[2] = error[1];
  error[1] = error[0];
  error[0] = setpoint - _state;

  ros::Duration delta_t;

  if(!prev_time.isZero()){
    delta_t = ros::Time::now() - prev_time;
    prev_time = ros::Time::now();
    if(delta_t.toSec() == 0 )
      return 0;
  }
  else{
    prev_time = ros::Time::now();
    return 0;
  }

  error_integral += error[0] * delta_t.toSec();

  if(fabs(error_integral) > fabs(windup_limit))
    error_integral = fabs(windup_limit) * (error_integral > 0 ? 1 : -1);

  if (cutoff_frequency != -1)
    {
      // Check if tan(_) is really small, could cause c = NaN
      tan_filt = tan( (cutoff_frequency*6.2832)*delta_t.toSec()/2 );

      // Avoid tan(0) ==> NaN
      if ( (tan_filt<=0.) && (tan_filt>-0.01) )
	tan_filt = -0.01;
      if ( (tan_filt>=0.) && (tan_filt<0.01) )
	tan_filt = 0.01;

      c = 1/tan_filt;
    }
  filtered_error[2] = filtered_error[1];
  filtered_error[1] = filtered_error[0];  
  filtered_error[0] = (1/(1+c*c+1.414*c))*(error[2]+2*error[1]+error[0]-(c*c-1.414*c+1)*filtered_error[2]-(-2*c*c+2)*filtered_error[1]);

  error_deriv[2] = error_deriv[1];
  error_deriv[1] = error_deriv[0];
  error_deriv[0] = (error[0]-error[1])/delta_t.toSec();

  filtered_error_deriv[2] = filtered_error_deriv[1];
  filtered_error_deriv[1] = filtered_error_deriv[0];
  if(loop_counter>2)
    filtered_error_deriv[0] = (1/(1+c*c+1.414*c))*(error_deriv[2]+2*error_deriv[1]+error_deriv[0]-(c*c-1.414*c+1)*filtered_error_deriv[2]-(-2*c*c+2)*filtered_error_deriv[1]);
  else
    loop_counter++;


  // calculate the control effort
  double proportional = Kp * filtered_error[0];
  double integral = Ki * error_integral;
  double derivative = Kd * filtered_error_deriv[0];
  double control_effort = proportional + integral + derivative;

  // Apply saturation limits
  if (control_effort > upper_limit)
    {
      control_effort = upper_limit;
    }
  else if (control_effort < lower_limit)
    {
      control_effort = lower_limit;
    }

  // Publish the stabilizing control effort if the controller is enabled

  // update diags
  return control_effort;
}
void PID::_reconfig_CB(alpha_autopilot::DynamicPIDConfig &config, uint32_t level){
  if (first_reconfig)
    {
      first_reconfig = false;
      return;     // Ignore the first call to reconfigure which happens at startup
    }

  Kp = config.Kp * config.Kp_scale;
  Ki = config.Ki * config.Ki_scale;
  Kd = config.Kd * config.Kd_scale;
  ROS_INFO("Pid reconfigure request: Kp: %f, Ki: %f, Kd: %f", Kp, Ki, Kd);

}
void PID::set_setpoint(double _setpoint){
  setpoint = _setpoint;
}
