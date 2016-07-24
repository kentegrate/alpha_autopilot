#ifndef ALPHA_SIMULATOR_CONTROL_H
#define ALPHA_SIMULATOR_CONTROL_H

class AircraftControl{
 public:
  double elevator;
  double rudder;
  double aileron;
  double throttle;//radians
  AircraftControl();
  AircraftControl(double e, double r, double a, double t);

  void print();
  void set(double e, double r, double a, double t);
  void calc_magnitude(double max_e, double min_e, 
		      double max_r, double min_r,
		      double max_a, double min_a,
		      double max_t, double min_t);

  const AircraftControl operator-(const AircraftControl &another);
};

#endif //ALPHA_SIMULATOR_CONTROL_H
