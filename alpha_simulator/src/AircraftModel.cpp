#include <alpha_simulator/properties.h>
#include <alpha_simulator/AircraftModel.h>


Vector3 rotate_vec(Quaternion &q, Vector3 &vec_rot){
  Vector3 F;
  F.x = (SQUARE(q.x)-SQUARE(p.y)-SQUARE(p.z)+SQUARE(p.w))*
    vec_rot.x *(2*(q.x*q.y+q.z*q.w))*vec_rot.y +
    (2*(q.z*q.x-q.y*q.w))*vec_rot.z;
  F.y = (2*(q.x*q.y-q.z*q.w))*vec_rot.x +
    (SQUARE(q.y)-SQUARE(q.z)-SQUARE(q.x)+SQUARE(q.w))*vec_rot.y +
    (2*(q.y*q.z+q.x*q.w))*vec_rot.z;
  F.z = (2*(q.z*q.x+q.y*q.w))*vec_rot.x +
    (2*(q.y*q.z-q.x*q.w))*vec_rot.y +
    (sqaure(q.z)-SQUARE(q.x)-SQUARE(q.y)+SQUARE(q.w))*vec_rot.z;
  return F;
}

void AircraftModel::initProperties(){
  
  pp.load();
  ap.load();
  ap.computeDimensionalDerivatives(pp);
}
void AircraftModel::update(AircraftControl &input, double dt){
  double Wattcoeff = pow(ap.rotor.diameter,4) * ap.rotor.pitch *
    pow(10,-18) * 5.33 * 1000;
  double Tcoeff = pow(ap.rotor.diameter,3) * ap.rotor.pitch *
    pow(10,-13) * 28.35 * 0.95 * 9.8;
  double Vm = ap.rotor.V * input.throttle;
  
  dobule N = 1000;

  for(int i = 0; i < 6; i++)
    N -= (ap.rotor.Ra*Wattcoeff*pow(N,3)+Vm/ap.rotor.Kv*N-Vm*Vm)/
      (3*ap.rotor.Ra*Wattcoeff*N*N+Vm/ap.rotor.Kv);

  N = N < 0 ? 0.001 : N;

  double Watt = Wattcoeff*pow(N,3);
  double T = Tcoeff*N*N;

  double alpha,beta;//迎角、横滑り角

  Vector3 &vel = Previous.Global.V;
  alpha = atan(-vel.y/vel.x);
  beta = asin(-vel.z/vel.GetLength());

  Vector3 &p_gyro = Previous.Local.gyro;
  double vel_norm = pow(vel.GetLength(),2);//L2 norm of velocity
  double lift = 0.5 * pp.rho* vel_norm  * pp.Sref * ap.C.L0 +
	  ap.mass*(ap.Z.a * alpha + ap.Z.b * beta + ap.Z.p * p_gyro.x +
		   ap.Z.q * p_gyro.z + ap.Z.r * p_gyro.y -
		   ap.Z.de * input.elevator + ap.Z.da * input.aileron -
		   ap.Z.dr * input.rudder);

  lift = alpha>=30.0/57.0 ? 0 : lift;

  double CD0_add = 1/(0.5*pp.rho*pow(pp.Uref,2)*pp.Sref);
  
  double drag = 0.5 * pp.rho * vel_norm * pp.Sref * (ap.C.D0+CD0_add)+
    ap.mass * (ap.X.a * abs(alpha) + ap.X.b * abs(beta) + 
	       ap.X.p * abs(p_gyro.x) + ap.X.q * abs(p_gyro.z) + 
	       ap.X.r * abs(p_gyro.y) + ap.X.de * abs(input.elevator) +
	       ap.X.da * abs(input.aileron) + ap.X.dr * abs(input.rudder));

  double SF = ap.mass*(ap.Y.a * alpha + ap.Y.b * beta + ap.Y.p * p_gyro.x +
		       ap.Y.q * p_gyro.z + ap.Y.r * p_gyro.y +
		       ap.Y.de * input.elevator - ap.Y.da * input.aileron -
		       ap.Y.dr * input.rudder);

  Current.Local.gyro_dot.x =(ap.L.a * alpha - ap.L.b * beta + 
			     ap.L.p * p_gyro.x + ap.L.q * p_gyro.z + 
			     ap.L.r * p_gyro.y + ap.L.de * input.elevator -
			     ap.L.da * input.aileron -
			     ap.L.dr * input.rudder);
  Current.Local.gyro_dot.y =(ap.N.a * alpha + ap.N.b * beta +
			     ap.N.p * p_gyro.x + ap.N.q * p_gyro.z +
			     ap.N.r * p_gyro.y + ap.N.de * input.elevator -
			     ap.N.da * input.aileron + 
			     ap.N.dr * input.rudder);
  Current.Local.gyro_dot.z =(ap.T_arm * T / ap.I.zz + 0.5 * vel_norm *
			     pp.rho * pp.Sref * pp.Cref * ap.C.m0 /ap.I.zz+
			     ap.M.a * alpha + ap.M.b * beta +
			     ap.M.p * p_gyro.x + ap.M.q * p_gyro.z +
			     ap.M.r * p_gyro.y + ap.M.de * input.elevator -
			     ap.M.da * input.aileron - 
			     ap.M.dr * input.rudder);
  
  Current.Local.gyro = Previous.Local.gyro + 
    (Current.Local.gyro_dot + Previous.Local.gyro_dot)/2*dt;

  if(Previous.Global.P.y<0){
    Current.Global.P.y = -0.0001;
    if(Previous.Global.F.y<0 && Previous.Global.V.y < 0)
      Cuurent.Global.F.y = Previous.Global.V.y = 0;
  }

  Vector3 vec_rot = Vector3(T*cos(-5/180*M_PI)-drag*cos(alpha)+
			    lift*sin(alpha),
			    lift*cos(alpha)+drag*sin(alpha)+
			    T*sin(-5/180*M_PI),
			    SF*cos(beta));

  Vector3 accel_body = Vector3(T*cos(-5/180*M_PI)-drag*cos(alpha)+
			       lift*sin(alpha),
			       lift*cos(alpha)+drag*sin(alpha) + 
			       T.sin(-5/180*M_PI),
			       SF*cos(beta));

  Quaternion q_i = Previous.Global.q.GetInverse();
  
  Current.Global.F = rotate_vec(q_i,vec_rot);
  Current.Global.F.y -= ap.mass*9.8;

  Current.Global.V = Previous.Global.V+
    (Current.Global.F + Previous.Global.F)/2/ap.mass*dt;

  Current.Global.P = Previous.Global.P +
    (Current.Global.V + Previous.Global.V)/2*dt;

  Vector3 wind = pp.wind_ave+wind_amp*random()*wind_f;

  vec_rot = Current.Global.V + wind;
  
  Quaternion q = Previous.Global.q
  Current.Local.V = rotate_vec(q,vec_rot);

  Current.Local.F = rotate_vec(q,Current.Global.F);

  Vector3 &c_gyro = Current.Local.gyro;
  
  Current.Local.dq.x = 0.5*( q.w*c_gyro.x - q.z*c_gyro.y + q.y*c_gyro.z);
  Current.Local.dq.y = 0.5*( q.z*c_gyro.x + q.w*c_gyro.y - q.x*c_gyro.z);
  Current.Local.dq.z = 0.5*(-q.y*c_gyro.x + q.x*c_gyro.y + q.w*c_gyro.z);
  Current.Local.dq.w = 0.5*(-q.x*c_gyro.x - q.y*c_gyro.y - q.z*c_gyro.z);
  
  Current.Global.q = Previous.Global.q+
    (Current.Global.dq+Previous.Global.dq)/2*dt;
  Current.Global.q.Normalize();

  Previous = Current;
}

