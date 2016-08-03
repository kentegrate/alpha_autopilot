
#include <alpha_autopilot/AlphaMode.h>
#include <cmath>
//TODO implement missions

AlphaState HorizontalTurn::get_setpoint(AlphaState state){
  AlphaState setpoint;
  setpoint.pos.z = initial_state.pos.z;
  setpoint.rot.x =10 * M_PI/180;
  return setpoint;
}

AlphaState EightTurn::get_setpoint(AlphaState state){
  AlphaState setpoint;
  setpoint.pos.z = initial_state.pos.z;
  setpoint.rot.x = 10 * M_PI/180;
  return setpoint;
}

AlphaState RiseTurn::get_setpoint(AlphaState state){
 AlphaState setpoint;
  setpoint.pos.z = initial_state.pos.z;
  setpoint.rot.x = 10 * M_PI/180;
  return setpoint;
}

AlphaState Glide::get_setpoint(AlphaState state){
  AlphaState setpoint;
  setpoint.pos.z = initial_state.pos.z;
  setpoint.rot.x = 10 * M_PI/180;
  return setpoint;
}

AlphaState Land::get_setpoint(AlphaState state){
  AlphaState setpoint;
  setpoint.pos.z = initial_state.pos.z;
  setpoint.rot.x = 10 * M_PI/180;
  return setpoint;
}



