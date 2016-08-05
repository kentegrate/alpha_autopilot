#ifndef ALPHA_LOCALIZATION_AHRS_NODELET_H
#define ALPHA_LOCALIZATION_AHRS_NODELET_H
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <alpha_localization/ahrs.h>
#include <alpha_msgs/IMU.h>
#include <geometry_msgs/Quaternion.h>
namespace alpha_autopilot{
 class AHRSNodelet : public nodelet::Nodelet{
    ros::Subscriber imu_sub;
    ros::Publisher orientation_pub;
    ros::Timer eventTimer;

    virtual void onInit();

    void imuCB(const alpha_msgs::IMU::ConstPtr msg);

    Ahrs ahrs;
    int calibration_counter;
    bool ahrs_ready;
    bool have_new_data;
    float gx,gy,gz,ax,ay,az,mx,my,mz;
    std::vector<float> offset;
 public:
    AHRSNodelet();
    void ahrsEvent(const ros::TimerEvent &msg);
  };



}


#endif //ALPHA_LOCALIZATION_AHRS_NODELET_H
