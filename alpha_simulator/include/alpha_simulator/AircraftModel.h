#ifndef ALPHA_SIMULATOR_AIRCRAFTMODEL_H
#define ALPHA_SIMULATOR_AIRCRAFTMODEL_H

#include <alpha_simulator/properties.h>
#include <alpha_simulator/control.h>

class AircraftModel{
public:
  PhysicalProperties pp;
  AircraftProperties ap;


  struct BiState{
    PhysicalState Global;
    PhysicalState Local;
  };

  BiState Current;
  BiState Previous;

  void initProperties();
  void getGZPose(gazebo::math::Pose &pose);
  void update(AircraftControl input,double dt);
  

};

#endif //ALPHA_SIMULATOR_AIRCRAFTMODEL_H
