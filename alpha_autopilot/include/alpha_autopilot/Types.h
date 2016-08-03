#ifndef ALPHA_AUTOPILOT_TYPES_H
#define ALPHA_AUTOPILOT_TYPES_H
struct Vector3{
  double x;
  double y;
  double z;
  Vector3(){
    x = 0;
    y = 0;
    z = 0;
  }
};
struct AlphaState{
  Vector3 pos;
  Vector3 rot;//.x->roll .y->pitch .z->yaw
};

#endif //ALPHA_AUTOPILOT_TYPES_H
