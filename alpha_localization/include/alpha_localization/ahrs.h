
#ifndef ALPHA_LOCALIZATION_MADGWICK_AHRS_H
#define ALPHA_LOCALIZATION_MADGWICK_AHRS_H

class Ahrs{
 public:
  volatile float q0,q1,q2,q3;

  Ahrs(float sample_hz = 512.0f ,float _beta = 0.1f,float _twoKi = 0.0f){
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
    beta = _beta;	// 2 * proportional gain
    sampleFreq = sample_hz;	// sample frequency in Hz
    twoKp = _beta;
    twoKi = _twoKi;

    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;    
    
  }
  
  void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
  void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

  void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

  void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
 private:
  float sampleFreq;
  volatile float beta;
  //for mahonyahrs
  volatile float twoKp,twoKi;
  volatile float integralFBx,integralFBy,integralFBz;
};

#endif //ALPHA_LOCALIZATION_MADGWICK_AHRS_H
