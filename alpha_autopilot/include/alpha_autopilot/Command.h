#ifndef ALPHA_AUTOPILOT_COMMAND_H
#define ALPHA_AUTOPILOT_COMMAND_H
#include <ros/ros.h>
#include <alpha_msgs/RC.h>

#define HIGH_THRESH 2000
#define LOW_THRESH 1000
enum struct AlphaCommand{
  NONE,LOW,MID,HIGH,
};
struct CMD{
  AlphaCommand ch1;
  AlphaCommand ch2;
  AlphaCommand ch3;
  AlphaCommand ch4;
  AlphaCommand ch5;
  AlphaCommand ch6;
  AlphaCommand ch7;
  AlphaCommand ch8;  
  
  CMD(AlphaCommand _ch1, AlphaCommand _ch2, AlphaCommand _ch3, AlphaCommand _ch4,
      AlphaCommand _ch5, AlphaCommand _ch6, AlphaCommand _ch7, AlphaCommand _ch8){
    ch1 = _ch1;
    ch2 = _ch2;
    ch3 = _ch3;
    ch4 = _ch4;
    ch5 = _ch5;
    ch6 = _ch6;
    ch7 = _ch7;
    ch8 = _ch8;
  }

  bool operator==(const CMD &that)const;
  std::vector<AlphaCommand> to_vector()const;

};

enum struct AlphaState{
  MANUAL,AUTO_HORIZONTAL_TURN,AUTO_EIGHT_TURN,
  AUTO_RISE_TURN,AUTO_GLIDE,AUTO_LANDING,SET_TRIM,
  SHUTDOWN,
};

class Command{
 public:
  void init();
 private:
  CMD descretizeInput(std::vector<int> &rc_raw);
  AlphaState decodeCommand(std::vector<int> &rc_raw);
  std::vector<int> executeCommand(AlphaState state, std::vector<int> &rc_raw);  
  void rcInputCB(alpha_msgs::RC::ConstPtr msg);
  void publishRC(std::vector<int> &rc_out);

  ros::NodeHandle nh;
  ros::Subscriber rc_sub;
  ros::Publisher rc_pub;
};


#endif //ALPHA_AUTOPILOT_COMMAND_H
