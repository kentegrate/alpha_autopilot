#include <alpha_autopilot/Command.h>
#include <unistd.h>
#include <linux/reboot.h>
#include <sys/reboot.h>

std::vector<AlphaCommand> CMD::to_vector()const{
  std::vector<AlphaCommand> v;
  v.push_back(ch1);
  v.push_back(ch2);
  v.push_back(ch3);
  v.push_back(ch4);
  v.push_back(ch5);
  v.push_back(ch6);
  v.push_back(ch7);
  v.push_back(ch8);
  return v;
}
bool CMD::operator==(const CMD &that)const{
  std::vector<AlphaCommand> this_cmd = to_vector();
  std::vector<AlphaCommand> that_cmd = that.to_vector();
  bool is_equal = true;
  for(int i = 0; i < this_cmd.size(); i++){
    is_equal &= (this_cmd[i] == AlphaCommand::NONE || 
		 that_cmd[i] == AlphaCommand::NONE ||
		 this_cmd[i] == that_cmd[i]);
  }
  return is_equal;

}

AlphaState Command::decodeCommand(std::vector<int> &rc_raw){
  CMD cmd = descretizeInput(rc_raw);
  const CMD MANUAL_CMD(AlphaCommand::NONE,AlphaCommand::NONE,AlphaCommand::NONE,AlphaCommand::NONE,
		       AlphaCommand::LOW,AlphaCommand::NONE,AlphaCommand::NONE,AlphaCommand::LOW);
  const CMD AUTO_HORIZONTAL_TURN_CMD(AlphaCommand::NONE,AlphaCommand::NONE,AlphaCommand::NONE,AlphaCommand::NONE,
				     AlphaCommand::HIGH,AlphaCommand::LOW,AlphaCommand::LOW,AlphaCommand::NONE);   
  const CMD AUTO_EIGHT_TURN_CMD(AlphaCommand::NONE,AlphaCommand::NONE,AlphaCommand::NONE,AlphaCommand::NONE,
				AlphaCommand::HIGH,AlphaCommand::LOW,AlphaCommand::MID,AlphaCommand::NONE);
  const CMD AUTO_RISE_TURN_CMD(AlphaCommand::NONE,AlphaCommand::NONE,AlphaCommand::NONE,AlphaCommand::NONE,
				AlphaCommand::HIGH,AlphaCommand::LOW,AlphaCommand::HIGH,AlphaCommand::NONE);
  const CMD AUTO_GLIDE_CMD(AlphaCommand::NONE,AlphaCommand::NONE,AlphaCommand::NONE,AlphaCommand::NONE,
			   AlphaCommand::HIGH,AlphaCommand::MID,AlphaCommand::LOW,AlphaCommand::NONE);
  const CMD AUTO_LANDING_CMD(AlphaCommand::NONE,AlphaCommand::NONE,AlphaCommand::NONE,AlphaCommand::NONE,
			   AlphaCommand::HIGH,AlphaCommand::MID,AlphaCommand::MID,AlphaCommand::NONE);
  const CMD SET_TRIM_CMD(AlphaCommand::NONE,AlphaCommand::NONE,AlphaCommand::NONE,AlphaCommand::NONE,
			 AlphaCommand::LOW,AlphaCommand::NONE,AlphaCommand::NONE,AlphaCommand::HIGH);
  const CMD SHUTDOWN_CMD(AlphaCommand::NONE,AlphaCommand::NONE,AlphaCommand::LOW,AlphaCommand::NONE,
			 AlphaCommand::HIGH,AlphaCommand::HIGH,AlphaCommand::HIGH,AlphaCommand::HIGH);

  if(MANUAL_CMD == cmd)
    return AlphaState::MANUAL;
  else if(AUTO_HORIZONTAL_TURN_CMD == cmd)
    return AlphaState::AUTO_HORIZONTAL_TURN;
  else if(AUTO_EIGHT_TURN_CMD == cmd)
    return AlphaState::AUTO_EIGHT_TURN;
  else if(AUTO_RISE_TURN_CMD == cmd)
    return AlphaState::AUTO_RISE_TURN;
  else if(AUTO_GLIDE_CMD == cmd)
    return AlphaState::AUTO_GLIDE;
  else if(AUTO_LANDING_CMD == cmd)
    return AlphaState::AUTO_LANDING;
  else if(SET_TRIM_CMD == cmd)
    return AlphaState::SET_TRIM;
  else if(SHUTDOWN_CMD == cmd)
    return AlphaState::SHUTDOWN;
  else 
    return AlphaState::MANUAL;
}
CMD Command::descretizeInput(std::vector<int> &rc_raw){
  std::vector<AlphaCommand> descretized;
  for(int i = 0; i < rc_raw.size(); i++){
    if(rc_raw[i] > HIGH_THRESH)
      descretized.push_back(AlphaCommand::HIGH);
    else if(rc_raw[i] < LOW_THRESH)
      descretized.push_back(AlphaCommand::LOW);      
    else
      descretized.push_back(AlphaCommand::MID);
  }
  CMD cmd(descretized[0],descretized[1],descretized[2],descretized[3],
	  descretized[4],descretized[5],descretized[6],descretized[7]);
  return cmd;
}
std::vector<int> Command::executeCommand(AlphaState state,std::vector<int> &rc_raw){
  std::vector<int> rc_out(8,0);
  if (state == AlphaState::MANUAL){
    rc_out = rc_raw;
  }
  else if(state == AlphaState::AUTO_HORIZONTAL_TURN){

  }
  else if(state == AlphaState::AUTO_EIGHT_TURN){

  }
  else if(state == AlphaState::AUTO_RISE_TURN){

  }
  else if(state == AlphaState::AUTO_GLIDE){

  }
  else if(state == AlphaState::AUTO_LANDING){

  }
  else if(state == AlphaState::SET_TRIM){
    //set trim for autopilot
    rc_out = rc_raw;
  }
  else if(state == AlphaState::SHUTDOWN){
    rc_sub.shutdown();
    sync();
    reboot(LINUX_REBOOT_CMD_HALT);
    
  }  
  else{
    rc_out = rc_raw;
  }
  
  return rc_out;
}

void Command::rcInputCB(alpha_msgs::RC::ConstPtr msg){
  if(msg->Channel.size() < 8)
    return;
  std::vector<int> rc_raw(8);
  for(int i = 0; i < 8; i++){
    rc_raw[i] = msg->Channel[i];
  }
  AlphaState state = decodeCommand(rc_raw);
  std::vector<int> rc_out = executeCommand(state,rc_raw);
  publishRC(rc_out);
}

void Command::publishRC(std::vector<int> &rc_out){
  alpha_msgs::RC msg;
  for(int i = 0; i < rc_out.size(); i++){
    msg.Channel.push_back(rc_out[i]);
  }
  rc_pub.publish(msg);

}
void Command::init(){
  rc_sub = nh.subscribe("/rc_raw",10,&Command::rcInputCB,this);
  rc_pub = nh.advertise<alpha_msgs::RC>("/rc_out",10);
}
