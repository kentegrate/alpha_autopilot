
#include <alpha_autopilot/AlphaMode.h>
#include <cmath>
#include <iostream>

static const float angle_error_range = 0.2;//rad
static const float landing_throttle_off_distance = 17;
static const float land_pos_z = 0.25;
static const float land_pos_x = 6.0;
static const float land_param = 1.0;
#define LAND_THROTTLE_ZERO_POINT_X -7
#define LAND_THROTTLE_ZERO_POINT_Z 1
#define LAND_THROTTLE_FALL_POINT_X -15

float add_angle(float ang1, float ang2){//add angles and 
  // return angles within range from -M_PI to M_PI
  float ans = ang1+ang2;
  while(fabsf(ans) > M_PI){
    ans += ans > 0 ? -2*M_PI : 2*M_PI;
  }
  return ans;
}
//check if the abs of the difference between the two 
//angles is within the error_range
bool in_range(float ang1, float ang2, float error_range){
  float error = fabsf(ang1-ang2);
  while(error>M_PI){
    error-=2*M_PI;
  }
  std::cout<<"error "<<error<<std::endl;

  return fabsf(error) < fabsf(error_range);
}
//TODO implement missions

AlphaState HorizontalTurn::get_setpoint(AlphaState state){
  AlphaState setpoint;
  setpoint.pos.z = initial_state.pos.z;
  setpoint.rot.x =0.4;
  is_initial = false;
  return setpoint;
}
float HorizontalTurn::get_throttle(){
  return initial_rc_in[THROTTLE_CH];
}

AlphaState EightTurn::get_setpoint(AlphaState state){
  AlphaState setpoint;
  setpoint.pos.z = initial_state.pos.z;
  float aim_rot_z = 0;
  if(phase == 0){//first half of the eight turn
    setpoint.rot.x = 0.4;
    aim_rot_z = add_angle(initial_state.rot.z,M_PI);
  }
  else if(phase == 1){
    setpoint.rot.x = 0.4;
    aim_rot_z = add_angle(initial_state.rot.z,0);
  }
  else{//other side of the eight turn
    setpoint.rot.x = -0.4;
  }
  if(in_range(aim_rot_z,state.rot.z,angle_error_range) && phase < 2)
    phase++;
  std::cout<<"phase "<<phase<<std::endl;
  std::cout<<"aim rot z "<< aim_rot_z<<std::endl;
  std::cout<<"rot z " <<state.rot.z<<std::endl;
  is_initial = false;
  return setpoint;
}
float EightTurn::get_throttle(){
  return initial_rc_in[THROTTLE_CH];
}


AlphaState RiseTurn::get_setpoint(AlphaState state){
 AlphaState setpoint;
 setpoint.rot.x = 0.4;
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
  setpoint.rot.x = 0.4;
  setpoint.rot.y = 0.1;
  is_initial = false;
  return setpoint;
}
float Glide::get_throttle(){
  return 0;
}


AlphaState Land::get_setpoint(AlphaState state){//marker state
  AlphaState setpoint;
  /*  if(fabsf(state.pos.x)<fabsf(LAND_THROTTLE_ZERO_POINT_X)){
    setpoint.rot.y = -3*M_PI/180;
    setpoint.pos.z = -1;//disabled
  }
  else{
    setpoint.rot.y = -1;//disabled
    float x0 = initial_state.pos.x;
    float z0 = initial_state.pos.z;
    float x1 = LAND_THROTTLE_ZERO_POINT_X;
    float z1 = LAND_THROTTLE_ZERO_POINT_Z;
    setpoint.pos.z = ((z0-z1)*state.pos.x+z1*x0-z0*x1)/(x0-x1);
    }*/
  setpoint.rot.y = 0;
    
  
  //  setpoint.rot.z = atan(state.pos.y/land_param);//this may need to be rotated
  
  setpoint.pos.y = 0;
  //according to the plus and minus of yaw.
  
  is_initial = false;
  current_state = state;
  return setpoint;
}
float Land::get_throttle(){
  float dist_marker = fabsf(current_state.pos.x);
  if(dist_marker < fabsf(LAND_THROTTLE_ZERO_POINT_X))
    return 0;
  else 
    return initial_rc_in[THROTTLE_CH];
  /*  else
    return (initial_rc_in[THROTTLE_CH])*
      (dist_marker-LAND_THROTTLE_ZERO_POINT_X)/
      (LAND_THROTTLE_FALL_POINT_X-LAND_THROTTLE_ZERO_POINT_X);*/
}



