#include <ros/ros.h>

#include <alpha_localization/ahrs.h>

#include <alpha_msgs/IMU.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
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
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/ahrs_imu",10);
  Ahrs ahrs(512,0.01,0.00001);
  ros::Rate rate(512);
  int count = 0;
  float offset[3];
  for(int i = 0; i < 3; i++)
    offset[i] = 0;
  while(count < 1000){
    if(have_new_data){
      gx *= 180/M_PI;
      gy *= 180/M_PI;
      gz *= 180/M_PI;
      offset[0] += (-gx*0.0175);
      offset[1] += (-gy*0.0175);
      offset[2] += (-gz*0.0175);
      count++;
      have_new_data=false;
    }
    rate.sleep();
    ros::spinOnce();
  }
  for(int i = 0; i< 3; i++)
    offset[i]/=1000;

  while(ros::ok()){
    if(have_new_data){
      //           ahrs.MadgwickAHRSupdate(gx,gy,gz,ax,ay,az,mx,my,mz);

      float costheta = -1;//around y axis theta=180deg
      float sintheta = 0;
      float cosphi = 0;//around z axis phi
      float sinphi = -1;
      float _mx = mx;
      float _my = my;
      float _mz = mz;

      /*      mx = cosphi*(_mx*costheta+_mz*sintheta)-_my*sinphi;
      my = sinphi*(_mx*costheta+_mz*sintheta)+_my*cosphi;
      mz = -_mx*sintheta+_mz*costheta;*/
      gx *= 180/M_PI;
      gy *= 180/M_PI;
      gz *= 180/M_PI;
      gx *= 0.0175;
      gy *= 0.0175;
      gz *= 0.0175;
      gx += offset[0];
      gy += offset[1];
      gz += offset[2];

      ax /= 9.80665;
      ay /= 9.80665;
      az /= 9.80665;

      ahrs.MahonyAHRSupdate(gx,gy,gz,ax,ay,az,my,mx,-mz);
      geometry_msgs::Quaternion msg;
      msg.x = (double)ahrs.q1;
      msg.y = (double)ahrs.q2;
      msg.z = (double)ahrs.q3;
      msg.w = (double)ahrs.q0;
      orientation_pub.publish(msg);

      geometry_msgs::PoseStamped pose_msg;//for rviz visualization
      pose_msg.header.stamp = ros::Time::now();
      pose_msg.header.frame_id = "/map";
      pose_msg.pose.orientation = msg;
      pose_pub.publish(pose_msg);
      have_new_data = false;
    }
    ros::spinOnce();
    rate.sleep();
  }
}  
