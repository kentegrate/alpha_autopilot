#include <ros/ros.h>

#include <alpha_localization/ahrs.h>

#include <alpha_msgs/IMU.h>
#include <geometry_msgs/Quaternion.h>
bool have_new_data = false;
float gx,gy,gz,ax,ay,az,mx,my,mz;

void imu_cb(const alpha_msgs::IMU::ConstPtr msg){
  have_new_data = true;
  ax = msg->linear_acceleration.x;
  ay = msg->linear_acceleration.y;
  az = msg->linear_acceleration.z;
  gx = msg->angular_velocity.x;
  gy = msg->angular_velocity.y;
  gz = msg->angular_velocity.z;
  mx = msg->magnetic_field.x;
  my = msg->magnetic_field.y;
  mz = msg->magnetic_field.z;
}
int main(int argc, char* argv[]){
  ros::init(argc, argv, "ahrs_node");
  ros::NodeHandle n;
  ros::Subscriber imu_sub = n.subscribe("/imu",10,imu_cb);
  ros::Publisher orientation_pub = n.advertise<geometry_msgs::Quaternion>("/ahrs",10);
  Ahrs ahrs(30);
  ros::Rate rate(30);
  while(ros::ok()){
    if(have_new_data){
      ahrs.MadgwickAHRSupdate(gx,gy,gz,ax,ay,az,mx,my,mz);
      geometry_msgs::Quaternion msg;
      msg.x = (double)ahrs.q1;
      msg.y = (double)ahrs.q2;
      msg.z = (double)ahrs.q3;
      msg.w = (double)ahrs.q0;
      orientation_pub.publish(msg);
      have_new_data = false;
    }
    ros::spinOnce();
    rate.sleep();
  }
}  
