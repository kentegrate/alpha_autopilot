#include <ros/ros.h>
#include <alpha_autopilot/Command.h>

int main(int argc, char* argv[]){
  ros::init(argc,argv,"command_node");
  Command cmd;
  cmd.init();
  ros::spin();
  return 0;
}
