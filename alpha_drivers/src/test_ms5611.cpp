
#include <ros/ros.h>
#include <alpha_drivers/ms5611.h>

int main(int argc, char* argv[]){
  ros::init(argc,argv,"test");
  ros::NodeHandle n;
  MS5611 ms5611;
  ms5611.initialize();
  while(ros::ok()){
    ms5611.update();
    std::cout<<"pressure "<<ms5611.getPressure()<<std::endl;
  }
  return 0;
}
