#include <alpha_autopilot/AlphaCommand.h>
#include <unistd.h>
#include <linux/reboot.h>
#include <sys/reboot.h>
#include <alpha_autopilot/Types.h>
#include <alpha_autopilot/AlphaMode.h>

int AlphaCommand::operator[](int index)const{
  return channels[index];
}

bool AlphaCommand::operator==(const AlphaCommand &that)const{

  bool is_equal = true;
  for(int i = 0; i < 8; i++){
    is_equal &= ((*this)[i] == LogicLevel::NONE || 
		 that[i] == LogicLevel::NONE ||
		 (*this)[i] == that[i]);
  }
  return is_equal;

}
AlphaMode* AlphaCommand::getMode(AlphaMode* current_mode, AlphaState current_state){

  std::vector<int> descretized;
  for(int i = 0; i < channels.size(); i++){
    if(channels[i] > HIGH_THRESH)
      descretized.push_back(LogicLevel::HIGH);
    else if(channels[i] < LOW_THRESH)
      descretized.push_back(LogicLevel::LOW);      
    else
      descretized.push_back(LogicLevel::MID);
  }
  
  AlphaCommand new_alpha_cmd(descretized);


  if(current_mode->getAlphaCommand() == new_alpha_cmd) //if current mode does not change
    return current_mode;

  delete current_mode;
  AlphaMode* new_mode;
  if(AlphaCommand::MANUAL_CMD() == new_alpha_cmd)
    new_mode = new ManualMode;
  else if(AlphaCommand::AUTO_HORIZONTAL_TURN_CMD() == new_alpha_cmd)
    new_mode = new HorizontalTurn(current_state,(*this));
  else if(AlphaCommand::AUTO_EIGHT_TURN_CMD() == new_alpha_cmd)
    new_mode = new EightTurn(current_state,(*this));
  else if(AlphaCommand::AUTO_RISE_TURN_CMD() == new_alpha_cmd)
    new_mode = new RiseTurn(current_state,(*this));
  else if(AlphaCommand::AUTO_GLIDE_CMD() == new_alpha_cmd)
    new_mode = new Glide(current_state,(*this));
  else if(AlphaCommand::AUTO_LANDING_CMD() == new_alpha_cmd)
    new_mode = new Land(current_state,(*this));
  else if(AlphaCommand::SET_TRIM_CMD() == new_alpha_cmd)
    new_mode = new SetTrim;
  else if(AlphaCommand::SHUTDOWN_CMD() == new_alpha_cmd){//shutdown
    sync();
    reboot(LINUX_REBOOT_CMD_HALT);
  }
  else 
    new_mode = new ManualMode;

  return new_mode;
}
