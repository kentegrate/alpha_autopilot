#include <alpha_localization/pose_nodelet.h>
#include <pluginlib/class_list_macros.h>


namespace alpha_autopilot{
  void PoseNodelet::onInit(){
    
    ros::NodeHandle &nh = getNodeHandle();
    ahrs_sub = nh.subscribe("/ahrs",10,&PoseNodelet::ahrs_cb,this);
    imu_sub = nh.subscribe("/imu",1,&PoseNodelet::imu_cb,this);
    baro_sub = nh.subscribe("/pressure",1,&PoseNodelet::baro_cb,this);
    calib_sub = nh.subscribe("/calibrate",1,&PoseNodelet::calib_cb,this);
    pose_pub = nh.advertise<alpha_msgs::FilteredState>("/pose",10);

    eventTimer = nh.createTimer(ros::Duration(0.01),&PoseNodelet::timerEvent,this);
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

    q_raw.w = 0;
  }

  void PoseNodelet::ahrs_cb(const geometry_msgs::QuaternionConstPtr msg){
    alpha_msgs::FilteredStatePtr pose(new alpha_msgs::FilteredState);
    float q0 = msg->w;
    float q1 = msg->x;
    float q2 = msg->y;
    float q3 = msg->z;
    q_raw = (*msg);
    pose->x = 0;
    pose->y = 0;
    pose->z = z;
    pose->roll = atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
    pose->pitch = asin(2*(q0*q2-q3*q1));
    pose->yaw = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
    pose_pub.publish(pose);
  }
  void PoseNodelet::imu_cb(const alpha_msgs::IMUConstPtr msg){
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
    float ax = msg->linear_acceleration.x;
    float ay = msg->linear_acceleration.y;
    float az = msg->linear_acceleration.z;
    float raw_az = 
      2*(q.x*q.z+q.w*q.y)*ax +
      2*(q.y*q.z-q.w*q.x)*ay +
      (1-2*q.x*q.x-2*q.y*q.y)*az;
    world_az = raw_az - az_offset;

    if(calibrating){
      az_count += 1;
      az_sum += raw_az;
    }
  }

  void PoseNodelet::baro_cb(const alpha_msgs::AirPressureConstPtr msg){

    float baro_raw = msg->pressure;

    if(calibrating){
      baro_count += 1;
      baro_sum += baro_raw;
    }
    else if(calibrated)
      baro_altitude = ((pow(p0/baro_raw,1/5.257)-1)*(temp+273.15))/0.0065-zero_altitude;

  }
  void PoseNodelet::calib_cb(const std_msgs::EmptyConstPtr msg){
    calibrating = true;
    calibration_start_time = ros::Time::now();
    baro_count = 0;
    baro_sum = 0;
    az_sum =  0;
    az_count = 0;
  }
  void PoseNodelet::timerEvent(const ros::TimerEvent &msg){
    if(calibrating){
      if(msg.current_real > calibration_start_time + calibration_duration){
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
      float dt = (msg.current_real-msg.last_real).toSec();
      vz += world_az*dt;
      z +=vz*dt;
      vz +=  k_vz*(z - baro_altitude);
      z += k_z*(z - baro_altitude);
    }
  }

  PLUGINLIB_DECLARE_CLASS(alpha_autopilot,PoseNodelet,alpha_autopilot::PoseNodelet, nodelet::Nodelet);
}
