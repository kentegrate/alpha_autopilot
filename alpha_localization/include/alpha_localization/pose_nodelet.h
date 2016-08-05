#ifndef ALPHA_LOCALIZATION_POSE_NODELET_H
#define ALPHA_LOCALIZATION_POSE_NODELET_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Quaternion.h>
#include <alpha_msgs/FilteredState.h>
#include <alpha_msgs/IMU.h>
#include <alpha_msgs/AirPressure.h>
#include <std_msgs/Empty.h>


namespace alpha_autopilot{
  class PoseNodelet : public nodelet::Nodelet{
    ros::Publisher pose_pub;
    ros::Subscriber ahrs_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber baro_sub;
    ros::Subscriber calib_sub;
    virtual void onInit();
    ros::Timer eventTimer;
    void ahrs_cb(const geometry_msgs::Quaternion::ConstPtr msg);
    void imu_cb(const alpha_msgs::IMU::ConstPtr msg);
    void baro_cb(const alpha_msgs::AirPressure::ConstPtr msg);
    void calib_cb(const std_msgs::Empty::ConstPtr msg);
    void timerEvent(const ros::TimerEvent &msg);

    ros::Duration calibration_duration;
    ros::Time calibration_start_time;
    float vz,z;
    bool calibrating;
    bool calibrated;
    int baro_count;
    bool baro_sum;
    float k_vz;
    float k_z;
    float baro_altitude;
    float zero_altitude;
    float temp;//C
    float p0;
    float world_az;
    float az_offset;
    float az_sum;
    int az_count;
    geometry_msgs::Quaternion q_raw;

  };

}


#endif //ALPHA_LOCALIZATION_POSE_NODELET_H
