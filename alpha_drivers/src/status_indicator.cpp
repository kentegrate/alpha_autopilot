#include <ros/ros.h>
#include <wiringPi.h>
#include <std_msgs/Empty.h>
#define READY_LED_PIN 24
#define CALIBRATE_LED_PIN 25
#define CALIBRATE_TIME 5000//in ms

std::vector<std::string> check_nodes = {"/rcout_node","/sbus_decoder","/spi_sensors"};

bool calibrating = false;
double calibrate_start_time = 0;


double get_dtime(void)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  //ミリ秒を計算
  return ((double)(tv.tv_sec)*1000 + (double)(tv.tv_usec)*0.001); //★
}

void calibrateCB(std_msgs::Empty::ConstPtr msg){
  calibrate_start_time = get_dtime();
  calibrating = true;
}

int main(int argc, char* argv[]){
  ros::init(argc,argv,"status_indicator");
  ros::NodeHandle nh;
  ros::Subscriber calibrate_sub = nh.subscribe("/calibrate",10,calibrateCB);
  if(wiringPiSetupGpio() == -1) return 1;
  pinMode(READY_LED_PIN,OUTPUT);
  pinMode(CALIBRATE_LED_PIN,OUTPUT);
  digitalWrite(CALIBRATE_LED_PIN,1);
  ros::Rate rate(1);
  bool initialize_done = false;
  while(ros::ok()){
    if(!initialize_done){
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
	initialize_done = true;
      }
    }

    if(calibrating){
      digitalWrite(CALIBRATE_LED_PIN,0);      
      if(get_dtime() > calibrate_start_time + CALIBRATE_TIME){
	calibrating = false;
	digitalWrite(CALIBRATE_LED_PIN,1);
      }
    }
    rate.sleep();    
  }
    
  return 0;

}
