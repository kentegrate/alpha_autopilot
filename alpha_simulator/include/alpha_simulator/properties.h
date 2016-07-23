#include <gazebo/MathTypes.hh>

typedef gazebo::math::Vector3 Vector3;
typedef gazebo::math::Quaternion Quaternion;

#define SQUARE(x) (x*x)

struct Inertia{
  double xx,xy,xz;
  double yx,yy,yz;
  double zx,zy,zz;
};
struct StabilityDerivatives{
  double a,b,p,q,r,de,da,dr;  
};

struct DimLessDerivatives{
  StabilityDerivatives L;  
  StabilityDerivatives D;  
  StabilityDerivatives C;  
  StabilityDerivatives l;  
  StabilityDerivatives m;  
  StabilityDerivatives n;  
  double L0,D0,m0;
};
struct DimensionalDerivatives{
}
class Properties{

};

class PhysicalProperties : public Properties{
 public:
  double Sref;//0.4739
  double Uref;//14.34
  double Cref;//0.271544
  double Bref;//1.8
  double rho;//1.225 the density of air

  //wind
  Vector3 wind_ave;
  Vector3 wind_amp;
  Vector3 wind_f;
};

class RotorProperties : public Properties{
 public:
  //propeller properties
  double diameter;
  double pitch;
  //motor properties
  double Kv;
  double Ra;
  double V;
  void load();
};
class AircraftProperties : public Properties{
 public:
  const double mass;//4.0
  //空力微係数  
  const Inertia I;
  double T_arm; //0.0
  //  Dimensionless derivatives
  DimLessDerivatives C;
  //  Dimensional Derivatives 
  StabilityDerivatives X;    
  StabilityDerivatives Y;    
  StabilityDerivatives Z;    
  StabilityDerivatives L;    
  StabilityDerivatives M;    
  StabilityDerivatives N;    
  RotorProperties rotor;
  void load();
  void computeDimensionalDerivatives(PhysicalProperties &pp);
};
//Current state and previous state needed
//also local and global state needed
class PhysicalState{
 public:
  Vector3 V;//velocity
  Vector3 gyro_dot;
  Vector3 F;//force
  Vector3 P;//position
  Vector3 gyro;
  Quaternion q;
  Quaternion dq;
};

