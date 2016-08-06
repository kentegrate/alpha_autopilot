#include <alpha_drivers/altitude_estimator.h>

void AltitudeEstimator::init(){
    
  calib_sub = nh.subscribe("/calibrate",1,&AltitudeEstimator::calib_cb,this);

  vz = 0;
  z = 0;
  calibrating = false;
  calibrated = false;
  baro_count = 0;
  baro_sum = 0;
  k_vz = -0.001;
  k_z = -0.035;
  zero_altitude = 20;
  temp = 30;
  az_offset = 0;
}



void AltitudeEstimator::calib_cb(const std_msgs::EmptyConstPtr msg){
  calibrating = true;
  calibration_start_time = ros::Time::now();
  baro_count = 0;
  baro_sum = 0;
  az_sum =  0;
  az_count = 0;
}
float AltitudeEstimator::getWorldZAccel(geometry_msgs::Quaternion &orientation, alpha_msgs::IMU &imu){
  //imu process
  geometry_msgs::Quaternion &q_raw = orientation;
  float q_norm = sqrt(q_raw.w*q_raw.w+q_raw.x*q_raw.x+q_raw.y*q_raw.y+q_raw.z*q_raw.z);
  //normalize q
  q_raw.w /= q_norm;
  q_raw.x /= q_norm;
  q_raw.y /= q_norm;
  q_raw.z /= q_norm;
  geometry_msgs::Quaternion q_inv;
  q_inv.w = q_raw.w;
  q_inv.x = -q_raw.x;
  q_inv.y = -q_raw.y;
  q_inv.z = -q_raw.z;
  geometry_msgs::Quaternion &q = q_inv;
  float ax = imu.linear_acceleration.x;
  float ay = imu.linear_acceleration.y;
  float az = imu.linear_acceleration.z;
  float raw_az = 
    2*(q.x*q.z+q.w*q.y)*ax +
    2*(q.y*q.z-q.w*q.x)*ay +
    (1-2*q.x*q.x-2*q.y*q.y)*az;
  return raw_az;
}

float AltitudeEstimator::update(geometry_msgs::Quaternion &orientation,alpha_msgs::IMU &imu, alpha_msgs::AirPressure &pressure){

  float baro_raw = pressure.pressure;
  float az = getWorldZAccel(orientation,imu);
  if(calibrating){

    az_count ++;
    az_sum += az;

    baro_count ++;
    baro_sum += baro_raw;

    if(ros::Time::now() > calibration_start_time + calibration_duration){
      float baro_mean = baro_sum/baro_count;
      std::cout<<"baro mean "<<baro_mean<<std::endl;
      p0 = baro_mean/pow((1-(0.0065*zero_altitude)/(temp+0.0065*zero_altitude+273.15)),5.257);
      std::cout<< "p0 "<<p0<<std::endl;
      az_offset = az_sum/az_count;
      calibrating=false;
      z = 0;
      vz = 0;
      calibrated = true;
    }
  }
  else if(calibrated){
    float world_az = az - az_offset;
    float dt = (ros::Time::now() - last_update).toSec();
    vz += world_az*dt;
    z +=vz*dt;

    float baro_altitude = ((pow(p0/baro_raw,1/5.257)-1)*(temp+273.15))/0.0065-zero_altitude;
    vz +=  k_vz*(z - baro_altitude);
    z += k_z*(z - baro_altitude);
  }

  last_update = ros::Time::now();
  return z;
}
