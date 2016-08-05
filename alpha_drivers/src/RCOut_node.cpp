#include <ros/ros.h>
#include <alpha_msgs/RC.h>
//#include <alpha_drivers/pca9685.h>
#include <alpha_drivers/PCA9685.h>
#include <wiringPi.h>
float freq = 60.0f;
bool need_init = true;
int  pulse[16];
Ada_ServoDriver pwm;

void rc_cb(alpha_msgs::RC::ConstPtr msg){
  freq =  msg->Frequency;
  
  if(need_init)
    return;
  for(int i = 0; i < msg->Channel.size();i++)
    pulse[i] = msg->Channel[i];
  //  for(int i = 0; i < 8; i++)
  //    pwm.setPWM(i+3,0,msg->Channel[i]);
}

int main(int argc, char* argv[]){
  ros::init(argc,argv,"rcout_node");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/rc_out",1,rc_cb);

  if(wiringPiSetupGpio() == -1) return 1;

  pinMode(27,OUTPUT);
  digitalWrite(27,0);
  
  pwm.init();
  pwm.reset();
  usleep(100000);
  pwm.setPWMFreq(freq);


  //pwm.setAllPWM(1000);


  need_init = false;
  ros::Rate rate(100);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
    for(int i = 0; i < 8; i++)
      pwm.setServoPulse(i+3,pulse[i]);
  }

  return 0;

}
