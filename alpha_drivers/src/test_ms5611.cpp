
#include <ros/ros.h>
#include <alpha_drivers/ms5611.h>

int main(int argc, char* argv[]){
  ros::init(argc,argv,"test_ms5611");
  ros::NodeHandle n;
  MS5611 ms5611;
  ms5611.initialize();
  ros::Rate rate(30);
  while(ros::ok()){
    ms5611.update();
    std::cout<<"pressure "<<ms5611.getPressure()<<std::endl;
    rate.sleep();
  }
  return 0;
}
