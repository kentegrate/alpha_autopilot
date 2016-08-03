#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <alpha_msgs/FilteredState.h>

ros::Publisher pose_pub;

void ahrs_cb(geometry_msgs::Quaternion::ConstPtr msg){
  alpha_msgs::FilteredState pose;
  double q0 = msg->w;
  double q1 = msg->x;
  double q2 = msg->y;
  double q3 = msg->z;
  pose.x = 0;
  pose.y = 0;
  pose.z = 0;
  pose.roll = atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
  pose.pitch = asin(2*(q0*q2-q3*q1));
  pose.yaw = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

  pose_pub.publish(pose);
}

int main(int argc, char* argv[]){
  ros::init(argc,argv,"simple_pose_node");

  ros::NodeHandle nh;
  ros::Rate rate(100);
  ros::Subscriber ahrs_sub = nh.subscribe("/ahrs",10,ahrs_cb);
  pose_pub = nh.advertise<alpha_msgs::FilteredState>("/pose",10);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

  return 0;

}
