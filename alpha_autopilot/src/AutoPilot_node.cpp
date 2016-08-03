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
  //  ros::NodeHandle nh;
  AutoPilot autopilot;
  autopilot.init();

  double last_time = get_dtime();
  while(ros::ok()){
    //    if(last_time + 10 < get_dtime()){
      autopilot.update();
      ros::spinOnce();
      //      last_time = get_dtime();
      //    }

  }
  return 0;
}
