
#include <alpha_autopilot/AlphaMode.h>
#include <cmath>

static const int angle_error_range = 0.2;//rad
static const float landing_throttle_off_distance = 17;
static const float land_pos_z = 0.25;
static const float land_pos_x = 6.0;
static const float land_param = 1.0;
float add_angle(float ang1, float ang2){//add angles and 
  // return angles within range from -M_PI to M_PI
  float ans = ang1+ang2;
  while(fabsf(ans) > M_PI){
    ans += ans > 0 ? -2*M_PI : 2*M_PI;
  }
  return ans
}
//check if the abs of the difference between the two 
//angles is within the error_range
bool in_range(float ang1, float ang2, float error_range){
  float error = fabsf(ang1-ang2);
  while(error>M_PI){
    error-=2*M_PI;
  }
  return fabsf(error) < fabsf(error_range);
}
//TODO implement missions

AlphaState HorizontalTurn::get_setpoint(AlphaState state){
  AlphaState setpoint;
  setpoint.pos.z = initial_state.pos.z;
  setpoint.rot.x =0.2;
  is_initial = false;
  return setpoint;
}
float HornizontalTurn::get_throttle(){
  return initial_rc_in[THROTTLE_CH];
}

AlphaState EightTurn::get_setpoint(AlphaState state){
  AlphaState setpoint;
  setpoint.pos.z = initial_state.pos.z;
  float aim_rot_z = 0;
  if(phase == 0){//first half of the eight turn
    setpoint.rot.x = 0.2;
    aim_rot_z = add_angle(initial_state.rot.z,M_PI);
  }
  else if(phase == 1){
    setpoint.rot.x = 0.2;
    aim_rot_z = add_angle(initial_state.rot.z,0);
  }
  else{//other side of the eight turn
    setpoint.rot.x = -0.2;
  }
  if(in_range(aim_rot_z,state.rot.z,angle_error_range) && phase < 2)
    phase++;

  is_initial = false;
  return setpoint;
}
float EightTurn::get_throttle(){
  return initial_rc_in[THROTTLE_CH];
}


AlphaState RiseTurn::get_setpoint(AlphaState state){
 AlphaState setpoint;
 setpoint.rot.x = 0.2;
 float aim_rot_z;
 if(phase==0){
   setpoint.pos.z = initial_state.pos.z;
   aim_rot_z = add_angle(initial_state.rot.z,M_PI);
 }
 else if(phase==1){
   setpoint.pos.z = initial_state.pos.z;
   aim_rot_z = add_angle(initial_state.rot.z,0);
 }
 else if(phase==2){
   setpoint.pos.z = initial_state.pos.z;
   aim_rot_z = add_angle(initial_state.rot.z,M_PI);
 }
 else if(phase==3){
   setpoint.pos.z = initial_state.pos.z;
   aim_rot_z = add_angle(initial_state.rot.z,0);
 }
 else{
   setpoint.pos.z = initial_state.pos.z+4;
 }
 if(in_range(aim_rot_z,state.rot.z,angle_error_range) && phase < 4)
   phase++;

  is_initial = false;
  return setpoint;
}
float RiseTurn::get_throttle(){
  return initial_rc_in[THROTTLE_CH];
}


AlphaState Glide::get_setpoint(AlphaState state){
  AlphaState setpoint;
  setpoint.rot.x = 0.2;
  setpoint.rot.y = 0;
  is_initial = false;
  return setpoint;
}
float Glide::get_throttle(){
  return 0;
}


AlphaState Land::get_setpoint(AlphaState state){//marker state
  AlphaState setpoint;
  float x0 = initial_state.pose.x;
  float z0 = initial_state.pose.z;
  float x1 = land_pos_x;
  float z1 = land_pos_z;
  setpoint.pos.z = ((z0-z1)*state.pos.x+z1*x0-z0*x1)/(x0-x1);
  
  setpoint.rot.z = atan(state.pos.y/land_param);//this may need to be rotated
  //according to the plus and minus of yaw.
  
  is_initial = false;
  current_state = state;
  return setpoint;
}
float Land::get_throttle(){
  float dist_marker = sqrt(pow(current_state.pos.x,2)+pow(current_state.pos.y,2));
  return (dist_marker > landing_throttle_off_distance) ? initial_rc_in[THROTTLE_CH] : 0;
}



