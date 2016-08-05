#include <alpha_autopilot/PID.h>

#include <ros/ros.h>

float Uniform( void ){
  return ((float)rand()+1.0)/((float)RAND_MAX+2.0);
}


int main(int argc, char* argv[]){
  ros::init(argc,argv,"pid_sample");

  ros::NodeHandle nh;
  PID pid("roll");
  pid.set_setpoint(5);
  float roll_state = -1;
  ros::Rate rate(30);
  while(ros::ok()){
    

    float effort = pid.update(roll_state);
    //    std::cout<<"effort "<<effort<<std::endl;
    roll_state += effort;
    roll_state += Uniform() - 0.5;
    std::cout<<"state " <<roll_state<<std::endl;
    rate.sleep();
    ros::spinOnce();
  }




}
