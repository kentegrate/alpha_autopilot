#include <alpha_simulator/properties.h>

class AircraftModel{
pubilc:
  PhysicalProperties pp;
  AircraftProperties ap;


  struct{
    PhysicalState Global;
    PhysicalState Local;
  } BiState;

  BiState Current;
  BiState Previous;
  void initProperites();
  void update(AircraftControl &input,double dt);
  

};

