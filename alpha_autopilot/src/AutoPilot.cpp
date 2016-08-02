#include <alpha_autopilot/AutoPilot.h>


AutoPilot::AutoPilot() : pid_roll("roll"),pid_pitch("pitch"),trim(8,0),rc_in(8,0){
  current_mode = new ManualMode;
  trim[0] = 1500;
  trim[ELEVATOR_CH] = 1500;
  trim[THROTTLE_CH] = 1100;
  trim[RUDDER_CH] = 1500;
}
AutoPilot::~AutoPilot(){
  delete current_mode;
}

void AutoPilot::update(){

  std::vector<int> rc_out;

  if(current_mode->isAuto()){
    AutoMode* automode = static_cast<AutoMode*>(current_mode);
    AlphaState setpoint;
    setpoint = automode->get_setpoint(state);

    pid_roll.set_setpoint(setpoint.rot.x);
    double roll_effort = pid_roll.update(state.rot.x);
    pid_pitch.set_setpoint(setpoint.rot.y);
    double pitch_effort = pid_pitch.update(state.rot.y);
    //    double throttle = automode->get_throttle();
    double throttle = rc_in[THROTTLE_CH];//for debug
    rc_out = compute_auto_rc_out(roll_effort,pitch_effort,throttle);//use trim 
    //turn on the LED on ch5,and ch2,ch3,ch4,ch5 is only available
    //TODO : throttle effort may be needed
    
  }
  else{//manual mode
    rc_out = compute_manual_rc_out(rc_in);
    //turn off the LED on ch5,and ch2,ch3,ch4,ch5 is only available
    if(current_mode->getAlphaCommand() == AlphaCommand::SET_TRIM_CMD())
      trim = rc_in;
  }

  publishRC(rc_out);

}
std::vector<int> AutoPilot::compute_auto_rc_out(double roll_effort,double pitch_effort,double throttle){
  std::vector<int> rc_out = trim;
  rc_out[AUTOPILOT_LED_CH] = 4096;
  rc_out[THROTTLE_CH] = throttle;
  rc_out[ELEVATOR_CH] += pitch_effort;// i think there needs to be a magnitude here
  rc_out[RUDDER_CH] += roll_effort;
  return rc_out;
}
std::vector<int> AutoPilot::compute_manual_rc_out(std::vector<int> rc_in){
  std::vector<int> rc_out= rc_in;
  rc_out[AUTOPILOT_LED_CH] = 0;
  return rc_out;
}

void AutoPilot::init(){
  rc_sub = nh.subscribe("/rc_in",10,&AutoPilot::rcInputCB,this);
  rc_pub = nh.advertise<alpha_msgs::RC>("/rc_out",10);
  state_sub = nh.subscribe("/pose",10,&AutoPilot::stateCB,this);
}
void AutoPilot::stateCB(alpha_msgs::FilteredState::ConstPtr msg){
  state.pos.x = msg->x;
  state.pos.y = msg->y;
  state.pos.z = msg->z;
  state.rot.x = msg->roll;
  state.rot.y = msg->pitch;
  state.rot.z = msg->yaw;
}
void AutoPilot::rcInputCB(alpha_msgs::RC::ConstPtr msg){
  if(msg->Channel.size() < 8)
    return;
  for(int i = 0; i < 8; i++){
    rc_in[i] = msg->Channel[i];
  }
  AlphaCommand cmd(rc_in);
  current_mode = cmd.getMode(current_mode,state);

}
void AutoPilot::publishRC(std::vector<int> &rc_out){
  alpha_msgs::RC msg;
  for(int i = 0; i < rc_out.size(); i++){
    msg.Channel.push_back(rc_out[i]);
  }
  rc_pub.publish(msg);

}
