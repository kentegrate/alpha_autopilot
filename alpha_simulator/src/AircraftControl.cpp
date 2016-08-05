#include <alpha_simulator/control.h>
#include <iostream>
#include <cmath>
AircraftControl::AircraftControl(){
  elevator = 0;
  rudder = 0;
  aileron = 0;
  throttle = 0;
}

AircraftControl::AircraftControl(double e, double r, double a, double t){
  elevator = e;
  rudder = r;
  aileron = a;
  throttle = t;
}
void AircraftControl::print(){
  std::cout<<"e "<<elevator<<std::endl;
  std::cout<<"r "<<rudder<<std::endl;
  std::cout<<"a "<<aileron<<std::endl;
  std::cout<<"t "<<throttle<<std::endl;
}
void AircraftControl::set(double e, double r, double a, double t){
  elevator = e;
  rudder = r;
  aileron = a;
  throttle = t;
}
void AircraftControl::calc_magnitude(double max_e, double min_e, 
				     double max_r, double min_r,
				     double max_a, double min_a,
				     double max_t, double min_t){


  elevator = elevator/(max_e-min_e)*M_PI/6;
  rudder   = rudder/(max_r-min_r)*M_PI/3;
  aileron  = aileron/(max_a-min_a);
  throttle =3*throttle/(max_t-min_t);
  std::cout<<"elevator "<<elevator<<" rudder "<<rudder<<" throttle "<<throttle<<std::endl;
}

const AircraftControl AircraftControl::operator-(const AircraftControl &another){
  AircraftControl ac;
  ac.elevator = this->elevator-another.elevator;
  ac.rudder = this->rudder-another.rudder;
  ac.aileron = this->aileron-another.aileron;
  ac.throttle = this->throttle-another.throttle;
  return ac;
}
