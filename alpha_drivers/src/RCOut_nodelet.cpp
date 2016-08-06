#include <alpha_drivers/RCOut_nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <wiringPi.h>

namespace alpha_autopilot{
  RCOut::~RCOut(){
    digitalWrite(27,1);//enable pwm output
  }
  void RCOut::onInit(){
    ros::NodeHandle &nh = getNodeHandle();
    sub = nh.subscribe("/rc_out",1,&RCOut::rc_sub,this);
    if(wiringPiSetupGpio() == -1) return;
    pinMode(27,OUTPUT);
    digitalWrite(27,0);//enable pwm output

    pwm.init();
    pwm.reset();

    usleep(100000);

    pwm.setPWMFreq(60);
    int pulse[8];
    for(int i = 0; i < 8; i++)
      pulse[i] = 0;
    set_pulse(pulse);
  }
  void RCOut::rc_sub(const alpha_msgs::RCConstPtr msg){
    if(msg->Channel.size() < 8)
      return;
    int pulse[8];
    for(int i = 0; i < 8;i++)
      pulse[i] =msg->Channel[i];
    set_pulse(pulse);
  }
  void RCOut::set_pulse(int* pulse){
    for(int i = 0; i < 8; i++)
      pwm.setServoPulse(i+3,pulse[i]);
  }
  


  PLUGINLIB_DECLARE_CLASS(alpha_autopilot,RCOut,alpha_autopilot::RCOut, nodelet::Nodelet);

}
