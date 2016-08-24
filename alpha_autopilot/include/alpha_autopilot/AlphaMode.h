#ifndef ALPHA_AUTOPILOT_ALPHAMODE_H
#define ALPHA_AUTOPILOT_ALPHAMODE_H

#define DROP_CH 0//this channel starts from 0
#define ELEVATOR_CH 1//this channel starts from 0
#define THROTTLE_CH 2
#define RUDDER_CH 3
#define AUTOPILOT_LED_CH 4
#define NEUTRAL_THROTTLE 1500
#define INITIAL_DROP_PULSE 1100
#define DROP_INC_PULSE 200
#define DROP_MAX_PULSE 1900
#define DROP_MIN_PULSE 1100

#include <alpha_autopilot/AlphaCommand.h>
#include <alpha_autopilot/Types.h>
class AlphaMode{
  /*  enum{
  MANUAL,AUTO_HORIZONTAL_TURN,AUTO_EIGHT_TURN,
  AUTO_RISE_TURN,AUTO_GLIDE,AUTO_LANDING,SET_TRIM,
  SHUTDOWN,
  }*/


 public:
  AlphaMode(){
  }
  
  virtual AlphaCommand getAlphaCommand()=0;
  virtual bool isAuto()= 0;
};

//concrete classes of manual modes
class ManualMode : public AlphaMode{
 public:
  ManualMode(){}
  AlphaCommand getAlphaCommand(){
    return AlphaCommand::MANUAL_CMD();
  }    
  bool isAuto(){
    return false;
  }
};
class Drop : public ManualMode{
 public:
  Drop(){
    is_initial = true;
  }
  AlphaCommand getAlphaCommand(){
    return AlphaCommand::DROP_CMD();
  }
  bool isInitial(){
    if(is_initial){
      is_initial = false;
      return true;
    }
    else
      return false;
  }
 private:
  bool is_initial;
};
class Calibrate : public ManualMode{
 public:
  Calibrate(){}
  AlphaCommand getAlphaCommand(){
    return AlphaCommand::CALIBRATE_CMD();
  }
};


class AutoMode : public AlphaMode{


 public:
  int phase;
  bool pid_reset = true;

  bool pid_z;
  bool pid_pitch;
  bool pid_yaw;
  bool pid_roll;
  bool pid_throttle;

  bool isInitial(){return pid_reset;}
  virtual AlphaState get_setpoint(AlphaState state)= 0;
  virtual float get_throttle() = 0;

  AutoMode(AlphaState _initial_state,AlphaCommand _initial_rc_in){
    initial_state = _initial_state;
    initial_rc_in = _initial_rc_in;
    phase = 0;
  }
  //  AlphaState get_setpoint(AlphaState state){} refresh current state and return setpoint
  // AlphaCommand getAlphaCommand(){}
  
  bool isAuto(){
    return true;
  }
  AlphaState initial_state;
  AlphaCommand initial_rc_in;
};

//concrete classes of auto modes
class HorizontalTurn : public AutoMode{
 public:
 HorizontalTurn(AlphaState _initial_state,AlphaCommand _initial_rc_in) : AutoMode(_initial_state, _initial_rc_in){

    pid_roll = true;
    pid_yaw = false;
    pid_z = true;
    pid_pitch = true;
    pid_throttle = false;

  }
  AlphaState get_setpoint(AlphaState state);
  float get_throttle();
  AlphaCommand getAlphaCommand(){
    return AlphaCommand::AUTO_HORIZONTAL_TURN_CMD();
  }
};
class EightTurn : public AutoMode{
 public:
  EightTurn(AlphaState _initial_state,AlphaCommand _initial_rc_in) : AutoMode(_initial_state, _initial_rc_in){
    pid_roll = true;
    pid_yaw = false;
    pid_z = true;
    pid_pitch = true;
    pid_throttle = false;

  }
  AlphaState get_setpoint(AlphaState state);
    float get_throttle();
  AlphaCommand getAlphaCommand(){  
    return AlphaCommand::AUTO_EIGHT_TURN_CMD();
  }
};
class RiseTurn : public AutoMode{
 public:
  RiseTurn(AlphaState _initial_state,AlphaCommand _initial_rc_in) : AutoMode(_initial_state, _initial_rc_in){
    pid_roll = true;
    pid_yaw = false;
    pid_z = true;
    pid_pitch = true;
    pid_throttle = false;
}
  AlphaState get_setpoint(AlphaState state);
    float get_throttle();
  AlphaCommand getAlphaCommand(){
    return AlphaCommand::AUTO_RISE_TURN_CMD();
  }

};
class Glide : public AutoMode{
 public:
  Glide(AlphaState _initial_state,AlphaCommand _initial_rc_in) : AutoMode(_initial_state, _initial_rc_in){
    pid_roll = true;
    pid_yaw = false;
    pid_z = false;
    pid_pitch = true;
    pid_throttle = false;

  }
  AlphaState get_setpoint(AlphaState state);
    float get_throttle();
  AlphaCommand getAlphaCommand(){
    return AlphaCommand::AUTO_GLIDE_CMD();
  }
};
class Land : public AutoMode{
 public:
  Land(AlphaState _initial_state,AlphaCommand _initial_rc_in) : AutoMode(_initial_state, _initial_rc_in){
    pid_roll = false;
    pid_yaw = true;
    pid_z = true;
    pid_pitch = true;
    pid_throttle = false;

}
  AlphaState get_setpoint(AlphaState state);
    float get_throttle();
  AlphaCommand getAlphaCommand(){
    return AlphaCommand::AUTO_LANDING_CMD();
  }
 private:
  AlphaState current_state;//used for get_throttle
};
#endif //ALPHA_AUTOPILOT_ALPHAMODE_H
