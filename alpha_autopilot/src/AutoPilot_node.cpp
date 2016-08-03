#include <ros/ros.h>
#include <alpha_autopilot/AutoPilot.h>


double get_dtime(void)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  //ミリ秒を計算
  return ((double)(tv.tv_sec)*1000 + (double)(tv.tv_usec)*0.001); //★
}

int main(int argc, char* argv[]){
  ros::init(argc,argv,"autopilot_node");
  ros::NodeHandle nh;
  AutoPilot autopilot;
  autopilot.init();
  ros::Rate rate(100);
  double last_time = get_dtime();
  while(ros::ok()){
      autopilot.update();

      ros::spinOnce();
      rate.sleep();
  }
  return 0;
}
