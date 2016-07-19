#include <ros/ros.h>
#include <alpha_msgs/RC.h>
#include <alpha_drivers/pca9685.h>

float freq = 73.5f;
bool need_init = true;

void rc_cb(alpha_msgs::RC::ConstPtr msg){
  freq =  msg->Frequency;
  
  if(need_init)
    return;

  for(int i = 0; i < msg->Channel.size(); i++)
    pwm.setPWM(i,0,msg->Channel[i]);



}

int main(int argc, char* argv[]){
  ros::init(argc,argv,"rcout_node");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/rc",10,rc_cb);

  pwm.setFrequency(freq);
  pwm.initialize();
  need_init = false;
  
  ros::spin();

  return 0;

}
