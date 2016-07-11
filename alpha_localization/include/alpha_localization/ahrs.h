
#ifndef ALPHA_LOCALIZATION_MADGWICK_AHRS_H
#define ALPHA_LOCALIZATION_MADGWICK_AHRS_H

class Ahrs{
 public:
  volatile float q0,q1,q2,q3;

  Ahrs(float sample_hz = 512.0f ,float _beta = 0.1f){
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
    beta = _beta;	// 2 * proportional gain
    sampleFreq = sample_hz;	// sample frequency in Hz
  }

  
  void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
 private:
  void _MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
  float sampleFreq;
  volatile float beta;


};

#endif //ALPHA_LOCALIZATION_MADGWICK_AHRS_H
