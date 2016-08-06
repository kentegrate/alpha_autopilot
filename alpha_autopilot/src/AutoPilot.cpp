#include <alpha_autopilot/AutoPilot.h>
#include <wiringPi.h>
#include <alpha_drivers/PCA9685.h>

AutoPilot::AutoPilot() : pid_roll("roll"),pid_pitch("pitch"),pid_z("z"),trim(8,0),rc_in(8,0){
  current_mode = new ManualMode;
  trim[0] = 1500;
  trim[ELEVATOR_CH] = 1500;
  trim[THROTTLE_CH] = 1100;
  trim[RUDDER_CH] = 1500;

  wiringPiSetupGpio();

  pinMode(27,OUTPUT);
  digitalWrite(27,0);
  
  pwm.init();
  pwm.reset();
  usleep(100000);
  pwm.setPWMFreq(60);
  
}
AutoPilot::~AutoPilot(){
  delete current_mode;
}

void AutoPilot::update(){

  std::vector<int> rc_out;

  if(current_mode->isAuto()){
    AutoMode* automode = static_cast<AutoMode*>(current_mode);
    AlphaState setpoint;
    if(automode->isInitial()){
      pid_roll.initialize();
      pid_pitch.initialize();
      pid_z.initialize();
    }

    setpoint = automode->get_setpoint(state);

    pid_roll.set_setpoint(setpoint.rot.x);
    float roll_effort = pid_roll.update(state.rot.x);

    if(current_mode->getAlphaCommand() != AlphaCommand::AUTO_EIGHT_TURN_CMD()){
      pid_z.set_setpoint(setpoint.pos.z);
      setpoint.rot.y = pid_z.update(state.pos.z);
    }
    
    pid_pitch.set_setpoint(setpoint.rot.y);
    float pitch_effort = pid_pitch.update(state.rot.y);
    //    float throttle = automode->get_throttle();
    float throttle = rc_in[THROTTLE_CH];//for debug
    rc_out = compute_auto_rc_out(roll_effort,pitch_effort,throttle);//use trim 
    if(current_mode->getAlphaCommand() == AlphaCommand::AUTO_HORIZONTAL_TURN_CMD()){
      rc_out[ELEVATOR_CH] = rc_in[ELEVATOR_CH];//for debug //automatic rudder
    }
    else if(current_mode->getAlphaCommand() == AlphaCommand::AUTO_EIGHT_TURN_CMD()){
      rc_out[RUDDER_CH] = rc_in[RUDDER_CH];//automatic elevator(keep pitch)
    }
    else if(current_mode->getAlphaCommand() == AlphaCommand::AUTO_RISE_TURN_CMD()){
      rc_out[RUDDER_CH] = rc_in[RUDDER_CH];//automatic elevator(keep altitude)
    }
    else if(current_mode->getAlphaCommand() == AlphaCommand::AUTO_GLIDE_CMD()){
      rc_out[RUDDER_CH] = rc_in[RUDDER_CH];//automatic elevator(keep initial_altitude +2 meters)
    }
    else if(current_mode->getAlphaCommand() == AlphaCommand::AUTO_LANDING_CMD()){
      //keep current altitude and turn
    }


    //turn on the LED on ch5,and ch2,ch3,ch4,ch5 is only available
    //TODO : throttle effort may be needed
    
  }
  else{//manual mode
    rc_out = compute_manual_rc_out(rc_in);
    //turn off the LED on ch5,and ch2,ch3,ch4,ch5 is only available
    if(current_mode->getAlphaCommand() == AlphaCommand::SET_TRIM_CMD())
      trim = rc_in;
    else if(current_mode->getAlphaCommand() == AlphaCommand::CALIBRATE_CMD())
      send_calibrate_request();
	    
  }

  setRCOut(rc_out);
  
}
std::vector<int> AutoPilot::compute_auto_rc_out(float roll_effort,float pitch_effort,float throttle){
  std::vector<int> rc_out = trim;
   rc_out[AUTOPILOT_LED_CH] = 4096;
    rc_out[THROTTLE_CH] = throttle;
    rc_out[ELEVATOR_CH] += pitch_effort*100;// i think there needs to be a magnitude here
    rc_out[RUDDER_CH] += roll_effort*100;
  return rc_out;
}
std::vector<int> AutoPilot::compute_manual_rc_out(std::vector<int> rc_in){
  std::vector<int> rc_out= rc_in;
  rc_out[AUTOPILOT_LED_CH] = 0;
  return rc_out;
}

void AutoPilot::init(){
  state_sub = nh.subscribe("/pose",10,&AutoPilot::stateCB,this);
  calibrate_pub = nh.advertise<std_msgs::Empty>("/calibrate",10);
}
void AutoPilot::stateCB(alpha_msgs::FilteredState::ConstPtr msg){

  state.pos.x = msg->x;
  state.pos.y = msg->y;
  state.pos.z = msg->z;
  state.rot.x = msg->roll;
  state.rot.y = msg->pitch;
  state.rot.z = msg->yaw;
}
void AutoPilot::setRCIn(std::vector<int> &rc_in){

  AlphaCommand cmd(rc_in);
  current_mode = cmd.getMode(current_mode,state);

}
void AutoPilot::setRCOut(std::vector<int> &rc_out){
  for(int i = 0; i < 8; i++)
    pwm.setServoPulse(i+3,rc_out[i]);
  //  nh.setParam("/rc_out",rc_out);
}
void AutoPilot::send_calibrate_request(){
  std_msgs::Empty msg;
  calibrate_pub.publish(msg);
}
