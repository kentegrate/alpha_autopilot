#include <alpha_simulator/properties.h>
#include <sstream>
#include<fstream>
#include<iostream>
#include<string>
#include <vector>
void RotorProperties::load(){
  diameter = 7;//inch
  pitch = 4;
  Kv = 1030;
  Ra = 0.15;
  V = 7.4;
}
void PhysicalProperties::load(){
  rho = 1.225;
}

void AircraftProperties::load(){

  rotor.load();
  
  std::ifstream ifs("/home/ken/UAVdata.csv");
  if(!ifs){
    std::cout<<"READ ERROR"<<std::endl;
    return;
  }

  std::string str;
  std::vector<double> aircraft_data;
  while(getline(ifs,str)){
    std::istringstream stream(str);
    double val;
    stream >> val;
    aircraft_data.push_back(val);
  }


  I.xx = 0.008445;
  I.yy = 0.01011;
  I.zz = 0.004132;
  
  C.L0 = aircraft_data[0];
  C.D0 = aircraft_data[1];
  C.m0 = aircraft_data[2];

  C.L.a = aircraft_data[3];
  C.D.a = aircraft_data[4];
  C.C.a = aircraft_data[5];
  C.l.a = aircraft_data[6];
  C.m.a = aircraft_data[7];
  C.n.a = aircraft_data[8];

  C.L.b = aircraft_data[9];
  C.D.b = aircraft_data[10];
  C.C.b = aircraft_data[11];
  C.l.b = -aircraft_data[12];
  C.m.b = aircraft_data[13];
  C.n.b = -aircraft_data[14];

  C.L.p = aircraft_data[15];
  C.D.p = aircraft_data[16];
  C.C.p = -aircraft_data[17];
  C.l.p = aircraft_data[18];
  C.m.p = aircraft_data[19];
  C.n.p = -aircraft_data[20];

  C.L.q = aircraft_data[21];
  C.D.q = aircraft_data[22];
  C.C.q = aircraft_data[23];
  C.l.q = aircraft_data[24];
  C.m.q = aircraft_data[25];
  C.n.q = aircraft_data[26];
  
  C.L.r = aircraft_data[27];
  C.D.r = aircraft_data[28];
  C.C.r = -aircraft_data[29];
  C.l.r = aircraft_data[30];
  C.m.r = aircraft_data[31];
  C.n.r = aircraft_data[32];

  C.L.de = aircraft_data[33];
  C.D.de = aircraft_data[34]; 
  C.C.de = aircraft_data[35];
  C.l.de = aircraft_data[36];
  C.m.de = aircraft_data[37];
  C.n.de = aircraft_data[38];

  C.L.da = aircraft_data[39];
  C.D.da = aircraft_data[40];
  C.C.da = aircraft_data[41];
  C.l.da = aircraft_data[42];
  C.m.da = aircraft_data[43];
  C.n.da = aircraft_data[44];

  C.L.dr = aircraft_data[45];
  C.D.dr = aircraft_data[46];
  C.C.dr = aircraft_data[47];
  C.l.dr = aircraft_data[48];
  C.m.dr = aircraft_data[49];
  C.n.dr = aircraft_data[50];

  T_arm = aircraft_data[51];

  mass = aircraft_data[52];

  Sref = aircraft_data[53];
  Bref = aircraft_data[54];  
  Cref = aircraft_data[55];
  Uref = aircraft_data[56];
}

void AircraftProperties::computeDimensionalDerivatives(PhysicalProperties &pp){
  double a_factor =pp.rho*Uref*Uref*Sref;
  X.a = a_factor/(2*mass)*C.D.a;
  Y.a = a_factor/(2*mass)*C.C.a;
  Z.a = a_factor/(2*mass)*C.L.a;
  L.a = a_factor*Bref/(2*I.xx)*C.l.a;
  M.a = a_factor*Cref/(2*I.zz)*C.m.a;
  N.a = a_factor*Bref/(2*I.yy)*C.n.a;

  double b_factor = pp.rho*Uref*Uref*Sref;
  X.b = b_factor/(2*mass)*C.D.b;
  Y.b = b_factor/(2*mass)*C.C.b;
  Z.b = b_factor/(2*mass)*C.L.b;
  L.b = b_factor*Bref/(2*I.xx)*C.l.b;
  M.b = b_factor*Cref/(2*I.zz)*C.m.b;
  N.b = b_factor*Bref/(2*I.yy)*C.n.b;

  double p_factor = pp.rho*Uref*Sref;
  X.p = p_factor * Cref/(4*mass)*C.D.p;
  Y.p = p_factor * Bref/(4*mass)*C.C.p;
  Z.p = p_factor * Cref/(4*mass)*C.L.p;
  L.p = p_factor * SQUARE(Bref)/(4*I.xx)*C.l.p;
  M.p = p_factor * SQUARE(Cref)/(4*I.zz)*C.m.p;
  N.p = p_factor * SQUARE(Bref)/(4*I.yy)*C.n.p;

  double q_factor = pp.rho*Uref*Sref;
  X.q = q_factor * Cref/(4*mass)*C.D.q;
  Y.q = q_factor * Bref/(4*mass)*C.C.q;
  Z.q = q_factor * Cref/(4*mass)*C.L.q;
  L.q = q_factor * SQUARE(Bref)/(4*I.xx)*C.l.q;
  M.q = q_factor * SQUARE(Cref)/(4*I.zz)*C.m.q;
  N.q = q_factor * SQUARE(Bref)/(4*I.yy)*C.n.q;

  double r_factor = pp.rho*Uref*Sref;
  X.r = r_factor * Cref / (4*mass)*C.D.r;
  Y.r = r_factor * Bref / (4*mass)*C.C.r;
  Z.r = r_factor * Cref / (4*mass)*C.L.r;
  L.r = r_factor * SQUARE(Bref)/(4*I.xx)*C.l.r;
  M.r = r_factor * SQUARE(Cref)/(4*I.zz)*C.m.r;
  N.r = r_factor * SQUARE(Bref)/(4*I.yy)*C.n.r;

  double de_factor = pp.rho*SQUARE(Uref)*Sref;
  X.de = de_factor /(2*mass)*C.D.de;
  Y.de = de_factor /(2*mass)*C.C.de;
  Z.de = de_factor /(2*mass)*C.L.de;
  L.de = de_factor * Bref/(2*I.xx)*C.l.de;
  M.de = de_factor * Cref/(2*I.zz)*C.m.de;
  N.de = de_factor * Bref/(2*I.yy)*C.n.de;

  double da_factor = pp.rho*SQUARE(Uref)*Sref;
  X.da = da_factor /(2*mass)*C.D.da;
  Y.da = da_factor /(2*mass)*C.C.da;
  Z.da = da_factor /(2*mass)*C.L.da;
  L.da = da_factor * Bref/(2*I.xx)*C.l.da;
  M.da = da_factor * Cref/(2*I.zz)*C.m.da;
  N.da = da_factor * Bref/(2*I.yy)*C.n.da;

  double dr_factor = pp.rho*SQUARE(Uref)*Sref;
  X.dr = dr_factor /(2*mass)*C.D.r;
  Y.dr = dr_factor /(2*mass)*C.C.dr;
  Z.dr = dr_factor /(2*mass)*C.L.r;
  L.dr = dr_factor * Bref/(2*I.xx)*C.l.dr;
  M.dr = dr_factor * Cref/(2*I.zz)*C.m.dr;
  N.dr = dr_factor * Bref/(2*I.yy)*C.n.dr;
  

}
