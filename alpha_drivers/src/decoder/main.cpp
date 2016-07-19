#include <alpha_drivers/decoder/GPIO_RPI.h>
#include <alpha_drivers/decoder/RCInput_RPI.h>
//#include <alpha_drivers/decoder/Scheduler.h>
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

  //  Scheduler scheduler;
  GPIO_RPI gpio;

  //  rcin.set_scheduler(&scheduler);
  rcin.set_gpio(&gpio);
  //  scheduler.setRCInput(&rcin);
  //  scheduler.init();
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
      //      print_ch(0);
      //      print_ch(1);
    }
    rate.sleep();
  }

  return 0;
}
