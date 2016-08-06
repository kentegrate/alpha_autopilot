#include <ros/ros.h>

#include <alpha_msgs/IMU.h>
#include <alpha_msgs/AirPressure.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>

class SensorRemapper{
 private:
  ros::NodeHandle nh;
  
  ros::Publisher imu_pub;
  ros::Publisher baro_pub;
  
  ros::Subscriber imu_sub;
  ros::Subscriber mag_sub;
  ros::Subscriber baro_sub;

  sensor_msgs::Imu imu_raw;
  geometry_msgs::Vector3 mag_raw;
  double baro_raw;
 public:
  void init();

  void imu_CB(sensor_msgs::ImuConstPtr msg);
  void mag_CB(geometry_msgs::Vector3StampedConstPtr msg);
  void baro_CB(geometry_msgs::PointStampedConstPtr msg);

  void publish_imu();
  void publish_baro();

};
