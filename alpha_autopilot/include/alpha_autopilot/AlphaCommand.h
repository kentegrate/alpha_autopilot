#ifndef ALPHA_AUTOPILOT_ALPHA_COMMAND_H
#define ALPHA_AUTOPILOT_ALPHA_COMMAND_H

#include <vector>

#define HIGH_THRESH 2000
#define LOW_THRESH 1110

class AlphaMode;
class AlphaState;

struct AlphaCommand{
  enum LogicLevel{
    NONE,LOW,MID,HIGH,
  };
  std::vector<int> channels;

  AlphaCommand static MANUAL_CMD(){
    AlphaCommand cmd(LogicLevel::NONE,LogicLevel::NONE,LogicLevel::NONE,LogicLevel::NONE,
		     LogicLevel::LOW,LogicLevel::NONE,LogicLevel::NONE,LogicLevel::LOW);
    return cmd;
  }
  AlphaCommand static AUTO_HORIZONTAL_TURN_CMD(){
    AlphaCommand cmd(LogicLevel::NONE,LogicLevel::NONE,LogicLevel::NONE,LogicLevel::NONE,
		     LogicLevel::HIGH,LogicLevel::LOW,LogicLevel::LOW,LogicLevel::NONE);   
    return cmd;
  }
  AlphaCommand static AUTO_EIGHT_TURN_CMD(){
    AlphaCommand cmd(LogicLevel::NONE,LogicLevel::NONE,LogicLevel::NONE,LogicLevel::NONE,
		     LogicLevel::HIGH,LogicLevel::LOW,LogicLevel::MID,LogicLevel::NONE);
    return cmd;
  }
  AlphaCommand static AUTO_RISE_TURN_CMD(){
    AlphaCommand cmd(LogicLevel::NONE,LogicLevel::NONE,LogicLevel::NONE,LogicLevel::NONE,
		     LogicLevel::HIGH,LogicLevel::LOW,LogicLevel::HIGH,LogicLevel::NONE);
    return cmd;
  }
  AlphaCommand static AUTO_GLIDE_CMD(){
    AlphaCommand cmd(LogicLevel::NONE,LogicLevel::NONE,LogicLevel::NONE,LogicLevel::NONE,
		     LogicLevel::HIGH,LogicLevel::MID,LogicLevel::LOW,LogicLevel::NONE);
    return cmd;
  }
  AlphaCommand static AUTO_LANDING_CMD(){
    AlphaCommand cmd(LogicLevel::NONE,LogicLevel::NONE,LogicLevel::NONE,LogicLevel::NONE,
		     LogicLevel::HIGH,LogicLevel::MID,LogicLevel::MID,LogicLevel::NONE);
    return cmd;
  }
  AlphaCommand static SET_TRIM_CMD(){
    AlphaCommand cmd(LogicLevel::NONE,LogicLevel::NONE,LogicLevel::NONE,LogicLevel::NONE,
		     LogicLevel::LOW,LogicLevel::NONE,LogicLevel::NONE,LogicLevel::HIGH);
    return cmd;
  }
  AlphaCommand static SHUTDOWN_CMD(){
    AlphaCommand cmd(LogicLevel::NONE,LogicLevel::NONE,LogicLevel::LOW,LogicLevel::NONE,
		     LogicLevel::HIGH,LogicLevel::HIGH,LogicLevel::HIGH,LogicLevel::HIGH);
    return cmd;
  }



AlphaCommand(std::vector<int> &input):channels(8,0){
  channels[0] = input[0];
  channels[1] = input[1];
  channels[2] = input[2];
  channels[3] = input[3];
  channels[4] = input[4];
  channels[5] = input[5];
  channels[6] = input[6];
  channels[7] = input[7];
}
AlphaCommand(int ch0, int ch1, int ch2, int ch3,int ch4, int ch5, int ch6, int ch7):channels(8,0){
  channels[0] = ch0;
  channels[1] = ch1;
  channels[2] = ch2;
  channels[3] = ch3;
  channels[4] = ch4;
  channels[5] = ch5;
  channels[6] = ch6;
  channels[7] = ch7;
}
AlphaCommand():channels(8,0){

}

  int operator[](int index) const;
  bool operator==(const AlphaCommand &that)const;

  AlphaMode* getMode(AlphaMode* current_mode, AlphaState current_state);

};




#endif //ALPHA_AUTOPILOT_ALPHA_COMMAND_H
