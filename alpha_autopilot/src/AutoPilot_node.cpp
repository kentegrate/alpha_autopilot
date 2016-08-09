#include <ros/ros.h>
#include <alpha_autopilot/AutoPilot.h>
#include <alpha_drivers/decoder/GPIO_RPI.h>
#include <alpha_drivers/decoder/RCInput_RPI.h>
#include <alpha_msgs/RC.h>
#include <fstream> 

RCInput_RPI rcin;
std::vector<int> pulse(LINUX_RC_INPUT_NUM_CHANNELS,0);
ros::Publisher rc_pub;
void termination_handler(int sig){
  rcin.deinit();
  rcin.running = false;
  ros::shutdown();
}

void rcInEvent(const ros::TimerEvent &msg, AutoPilot *autopilot){
  rcin._timer_tick();
  if(rcin.new_input()){
    alpha_msgs::RC msg;
    for(int i = 0; i < LINUX_RC_INPUT_NUM_CHANNELS; i++){
      pulse[i] = rcin.read(i);
      msg.Channel.push_back(pulse[i]);
    }
    rc_pub.publish(msg);
    autopilot->setRCIn(pulse);
  }
}
void autopilotEvent(const ros::TimerEvent &msg, AutoPilot *autopilot){
  autopilot->update();
}

int main(int argc, char* argv[]){

  ros::init(argc,argv,"autopilot_node",ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  rc_pub = nh.advertise<alpha_msgs::RC>("/rc_in",10);
  AutoPilot autopilot;
  autopilot.init();

  GPIO_RPI gpio;

  rcin.set_gpio(&gpio);
  for(int i = 0; i < 64; i++){
    struct sigaction sa;
    memset(&sa,0,sizeof(sa));
    sa.sa_handler = termination_handler;
    sigaction(i,&sa,NULL);
  }
  

  ros::Timer rcin_timer = nh.createTimer(ros::Duration(0.002),boost::bind(rcInEvent,_1,&autopilot));

  ros::Timer autopilot_timer = nh.createTimer(ros::Duration(0.01),boost::bind(autopilotEvent,_1,&autopilot));

  ros::Rate rate(500);

  while(rcin.ok()){
    ros::spinOnce();
    rate.sleep();
  }


  return 0;
}
