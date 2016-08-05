#include <alpha_localization/ahrs_nodelet.h>
#include <pluginlib/class_list_macros.h>


namespace alpha_autopilot{
  AHRSNodelet::AHRSNodelet():offset(3,0),ahrs(100,0.5,0.001){
  }
  void AHRSNodelet::onInit(){
    ahrs_ready = false;
    have_new_data = false;

    ros::NodeHandle &nh = getNodeHandle();
    imu_sub = nh.subscribe("/imu",10,&AHRSNodelet::imuCB,this);
    orientation_pub = nh.advertise<geometry_msgs::Quaternion>("/ahrs",10);
    eventTimer = nh.createTimer(ros::Duration(0.01),&AHRSNodelet::ahrsEvent,this);
  }
  void AHRSNodelet::ahrsEvent(const ros::TimerEvent &msg){
    if(!have_new_data)
      return;

    if(!ahrs_ready){//calibration
      offset[0] = -gx;
      offset[1] = -gy;
      offset[2] = -gz;
      calibration_counter++;

      if(calibration_counter >= 200){
	for(int i = 0; i < 3; i++)
	  offset[i]/=calibration_counter;

	ahrs_ready = true;
      }
    }
    else{
      gx += offset[0];
      gy += offset[1];
      gz += offset[2];
      ahrs.MadgwickAHRSupdate(gx,gy,gz,ax,ay,az,my,mx,-mz);
      geometry_msgs::QuaternionPtr msg(new geometry_msgs::Quaternion);
      msg->x = (double)ahrs.q1;
      msg->y = (double)ahrs.q2;
      msg->z = (double)ahrs.q3;
      msg->w = (double)ahrs.q0;
      orientation_pub.publish(msg);

    }
      have_new_data = false;
  }
  void AHRSNodelet::imuCB(const alpha_msgs::IMU::ConstPtr msg){
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
  PLUGINLIB_DECLARE_CLASS(alpha_autopilot,AHRSNodelet,alpha_autopilot::AHRSNodelet, nodelet::Nodelet);

}

