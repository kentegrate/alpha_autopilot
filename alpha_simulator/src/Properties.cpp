#include <alpha_simulator/properties.h>
void RotorProperties::load(){
  diameter = 7;//inch
  pitch = 4;
  Kv = 1030;
  Ra = 0.15;
  V = 7.4;
}
void PhysicalProperties::load(){
  Sref = 0.4739;
  Uref = 14.34;
  Cref = 0.271544;
  Bref = 1.8;
  rho = 1.225;
}

void AircraftProperties::load(){

  rotor.load();

  I.xx = 0.008445;
  I.yy = 0.01011;
  I.zz = 0.004132;
  
  C.L0 = 0;
  C.D0 = 0;
  C.mp = 0;

  C.L.a = -4.74;
  C.D.a = 0.038;
  C.C.a = -4.74;
  C.l.a = -4.74;
  C.m.a = -0.5;
  C.n.a = -4.74;

  C.L.b = -4.74;
  C.D.b = 0.038;
  C.C.b = -4.74;
  C.l.b = -4.74;
  C.m.b = -0.5;
  C.n.b = -4.74;

  C.L.p = -4.74;
  C.D.p = 0.038;
  C.C.p = -4.74;
  C.l.p = -4.74;
  C.m.p = -0.5;
  C.n.p = -4.74;

  C.L.q = -5.90;
  C.D.q = -5.90;
  C.C.q = -5.90;
  C.l.q = -5.90;
  C.m.q = -6.4;
  C.n.q = -6.4;
  
  C.L.r = -4.74;
  C.D.r = 0.038;
  C.C.r = -4.74;
  C.l.r = -4.74;
  C.m.r = -0.5;
  C.n.r = -4.74;

  C.L.de = -4.74;
  C.D.de = 0.038;
  C.C.de = -4.74;
  C.l.de = -4.74;
  C.m.de = -0.5;
  C.n.de = -4.74;

  C.L.da = -4.74;
  C.D.da = 0.038;
  C.C.da = -4.74;
  C.l.da = -4.74;
  C.m.da = -0.5;
  C.n.da = -4.74;

  C.L.dr = -4.74;
  C.D.dr = 0.038;
  C.C.dr = -4.74;
  C.l.dr = -4.74;
  C.m.dr = -0.5;
  C.n.dr = -4.74;

}

void AircraftProperties::computeDimensionalDerivatives(PhysicalProperties &pp){
  double a_factor =pp.rho*pp.Uref*pp.Uref*pp.Sref;
  X.a = a_factor/(2*mass)*C.D.a;
  Y.a = a_factor/(2*mass)*C.C.a;
  Z.a = a_factor/(2*mass)*C.L.a;
  L.a = a_factor*pp.Bref/(2*I.xx)*C.l.a;
  M.a = a_factor*pp.Cref/(2*I.zz)*C.m.a;
  N.a = a_factor*pp.Bref/(2*I.yy)*C.n.a;

  double b_factor = pp.rho*pp.Uref*pp.Uref*pp.Sref;
  X.b = b_factor/(2*mass)*C.D.b;
  Y.b = b_factor/(2*mass)*C.C.b;
  Z.b = b_factor/(2*mass)*C.L.b;
  L.b = b_factor*pp.Bref/(2*I.xx)*C.l.b;
  M.b = b_factor*pp.Cref/(2*I.zz)*C.m.b;
  N.b = b_factor*pp.Bref/(2*I.yy)*C.n.b;

  double p_factor = pp.rho*pp.Uref*pp.Sref;
  X.p = p_factor * pp.Cref/(4*mass)*C.D.p;
  Y.p = p_factor * pp.Bref/(4*mass)*C.C.p;
  Z.p = p_factor * pp.Cref/(4*mass)*C.L.p;
  L.p = p_factor * SQUARE(pp.Bref)/(4*I.xx)*C.l.p;
  M.p = p_factor * SQUARE(pp.Cref)/(4*I.zz)*C.m.p;
  N.p = p_factor * SQUARE(pp.Bref)/(4*I.yy)*C.n.p;

  double q_factor = pp.rho*pp.Uref*pp.Sref;
  X.q = q_factor * pp.Cref/(4*mass)*C.D.q;
  Y.q = q_factor * pp.Bref/(4*mass)*C.C.q;
  Z.q = q_factor * pp.Cref/(4*mass)*C.L.q;
  L.q = q_factor * SQUARE(pp.Bref)/(4*I.xx)*C.l.q;
  M.q = q_factor * SQUARE(pp.Cref)/(4*I.zz)*C.m.q;
  N.q = q_factor * SQUARE(pp.Bref)/(4*I.yy)*C.n.q;

  double r_factor = pp.rho*pp.Uref*pp.Sref;
  X.r = r_factor * pp.Cref / (4*mass)*C.D.r;
  Y.r = r_factor * pp.Bref / (4*mass)*C.C.r;
  Z.r = r_factor * pp.Cref / (4*mass)*C.L.r;
  L.r = r_factor * SQUARE(pp.Bref)/(4*I.xx)*C.l.r;
  M.r = r_factor * SQUARE(pp.Cref)/(4*I.zz)*C.m.r;
  N.r = r_factor * suqare(pp.Bref)/(4*I.yy)*C.n.r;

  double de_factor = pp.rho*SQUARE(pp.Uref)*pp.Sref;
  X.de = de_factor /(2*mass)*C.D.de;
  Y.de = de_factor /(2*mass)*C.C.de;
  Z.de = de_factor /(2*mass)*C.L.de;
  L.de = de_factor * pp.Bref/(2*I.xx)*C.l.de;
  M.de = de_factor * pp.Cref/(2*I.zz)*C.m.de;
  N.de = de_factor * pp.Bref/(2*I.yy)*C.n.de;

  double da_factor = pp.rho*SQUARE(pp.Uref)*pp.Sref;
  X.da = da_factor /(2*mass)*C.D.da;
  Y.da = da_factor /(2*mass)*C.C.da;
  Z.da = da_factor /(2*mass)*C.L.da;
  L.da = da_factor * pp.Bref/(2*I.xx)*C.l.da;
  M.da = da_factor * pp.Cref/(2*I.zz)*C.m.da;
  N.da = da_factor * pp.Bref/(2*I.yy)*C.n.da;

  double dr_factor = pp.rho*SQUARE(pp.Uref)*pp.Sref;
  X.dr = dr_factor /(2*mass)*C.D.r;
  Y.dr = dr_factor /(2*mass)*C.C.dr;
  Z.dr = dr_factor /(2*mass)*C.L.r;
  L.dr = dr_factor * pp.Bref/(2*I.xx)*C.l.dr;
  M.dr = dr_factor * pp.Cref/(2*I.zz)*C.m.dr;
  N.dr = dr_factor * pp.Bref/(2*I.yy)*C.n.dr;
  

}
