#include <ros/ros.h>
#include <wiringPi.h>
#define READY_LED_PIN 24

std::vector<std::string> check_nodes = {"/rcout_node","/sbus_decoder","/spi_sensors"};

int main(int argc, char* argv[]){
  ros::init(argc,argv,"status_indicator");
  ros::NodeHandle nh;
  
  if(wiringPiSetupGpio() == -1) return 1;
  pinMode(READY_LED_PIN,OUTPUT);
  ros::Rate rate(1);
  while(ros::ok()){
    int ok_nodes = 0;    
    std::vector<std::string> nodes;
    ros::master::getNodes(nodes);

    for(int i = 0; i <check_nodes.size(); i++){
      for(int j = 0; j < nodes.size(); j++){
	if(check_nodes[i] == nodes[j])
	  ok_nodes++;
      }
    }
    if(ok_nodes == check_nodes.size()){
      digitalWrite(READY_LED_PIN,0);
      break;
    }
    rate.sleep();    
  }
    
  return 0;

}
