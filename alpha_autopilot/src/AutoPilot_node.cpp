#include <ros/ros.h>
#include <alpha_autopilot/AutoPilot.h>

int main(int argc, char* argv[]){
  ros::init(argc,argv,"autopilot_node");
  ros::NodeHandle nh;
  AutoPilot autopilot;
  autopilot.init();
  ros::Rate rate(100);
  while(ros::ok()){
    autopilot.update();
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
