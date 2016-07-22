#include <alpha_drivers/decoder/GPIO_RPI.h>
#include <alpha_drivers/decoder/RCInput_RPI.h>
#include <alpha_msgs/RC.h>
#include <ros/ros.h>
RCInput_RPI rcin;

void termination_handler(int sig){
  rcin.deinit();
  rcin.running = false;
  ros::shutdown();
}
void print_ch(int ch){
  std::cout<<"ch "<<ch<<": "<<rcin.read(ch)<<std::endl;
}
int main(int argc, char* argv[]){
  ros::init(argc,argv,"sbus_decoder",ros::init_options::NoSigintHandler);
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<alpha_msgs::RC>("/rc",10);

  GPIO_RPI gpio;
  rcin.set_gpio(&gpio);

  for(int i = 0; i < 64; i++){
    struct sigaction sa;
    memset(&sa,0,sizeof(sa));
    sa.sa_handler = termination_handler;
    sigaction(i,&sa,NULL);
  }
  gpio.init();
  rcin.init();
  //  scheduler.system_initialized();
  ros::Rate rate(2000);
  while(rcin.ok()){
    rcin._timer_tick();
    if(rcin.new_input()){
      alpha_msgs::RC msg;
      msg.Frequency = 73.5f;
      for(int i = 0; i < LINUX_RC_INPUT_NUM_CHANNELS; i++)
	msg.Channel.push_back(rcin.read(i));
      
      pub.publish(msg);
    }
    rate.sleep();
  }

  return 0;
}
