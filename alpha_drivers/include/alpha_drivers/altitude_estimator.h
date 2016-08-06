#ifndef ALPHA_DRIVERS_ALTITUDE_ESTIMATOR_H
#define ALPHA_DRIVERS_ALTITUDE_ESTIMATOR_H

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <alpha_msgs/IMU.h>
#include <alpha_msgs/AirPressure.h>
#include <std_msgs/Empty.h>



class AltitudeEstimator{
 public:
 AltitudeEstimator():calibration_duration(3){}
  
  void init();

  float update(const geometry_msgs::QuaternionConstPtr orientation,const alpha_msgs::IMUConstPtr imu, const alpha_msgs::AirPressureConstPtr pressure);

 private:

  void calib_cb(const std_msgs::EmptyConstPtr msg);
  float getWorldZAccel(const geometry_msgs::QuaternionConstPtr orientation, const alpha_msgs::IMUConstPtr imu);

  ros::Subscriber calib_sub;
  ros::NodeHandle nh;

  ros::Duration calibration_duration;
  ros::Time calibration_start_time;
  ros::Time last_update;
  float vz,z;
  bool calibrating;
  bool calibrated;
  int baro_count;
  float baro_sum;
  float k_vz;
  float k_z;
  float zero_altitude;
  float temp;//C
  float p0;
  float az_offset;
  float az_sum;
  int az_count;

};




#endif //ALPHA_DRIVERS_ALTITUDE_ESTIMATOR_H
