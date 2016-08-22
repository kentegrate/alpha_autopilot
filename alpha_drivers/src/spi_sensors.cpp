
#include <ros/ros.h>
#include <alpha_drivers/ms5611.h>
#include <alpha_drivers/mpu9250.h>
#include <alpha_drivers/ahrs.h>
#include <alpha_drivers/altitude_estimator.h>

#include <alpha_msgs/IMU.h>
#include <alpha_msgs/AirPressure.h>
#include <alpha_msgs/FilteredState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <std_msgs/Empty.h>

#define STATE_INITIAL 1
#define STATE_WAIT_PRESSURE 2
#define STATE_WAIT_TEMPERATURE 3
bool calibrating = false;
double calib_start_time = 0;
float yaw_sum= 0;
int calib_count=0;
float yaw_offset = 0;
float add_angle(float ang1, float ang2){//add angles and 
  // return angles within range from -M_PI to M_PI
  float ans = ang1+ang2;
  while(fabsf(ans) > M_PI){
    ans += ans > 0 ? -2*M_PI : 2*M_PI;
  }
  return ans;
}

double get_dtime(void)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  //ミリ秒を計算
  return ((double)(tv.tv_sec)*1000 + (double)(tv.tv_usec)*0.001); //★
}
geometry_msgs::Vector3 float2VectorMsg(float* data){
  geometry_msgs::Vector3 msg;
  msg.x = (double)data[0];
  msg.y = (double)data[1];
  msg.z = (double)data[2];
  return msg;
}
double last_time;
void calibCB(const std_msgs::Empty::ConstPtr msg){
  calib_start_time = get_dtime();
  calibrating = true;
  yaw_sum = 0;
  calib_count = 0;
}
bool updatePressure(int &state, MS5611 &barometer,float &pressure){
  switch(state){
  case STATE_INITIAL:
    barometer.refreshPressure();
    last_time = get_dtime();
    state = STATE_WAIT_PRESSURE;
    break;
  case STATE_WAIT_PRESSURE:
    if(get_dtime()-last_time > 10){
      barometer.readPressure();
      barometer.refreshTemperature();
      state = STATE_WAIT_TEMPERATURE;
      last_time = get_dtime();
    }
    break;
  case STATE_WAIT_TEMPERATURE:
    if(get_dtime()-last_time > 10){
      barometer.readTemperature();
      barometer.calculatePressureAndTemperature();
      barometer.refreshPressure();
      last_time = get_dtime();
      state = STATE_WAIT_PRESSURE;
    }
    break;
  }
  float old_pressure = pressure;
  pressure = barometer.getPressure();
  return old_pressure != pressure;
}
int main(int argc, char* argv[]){
  ros::init(argc, argv, "spi_sensor_node");
  ros::NodeHandle nh;

  ros::Publisher imu_pub = nh.advertise<alpha_msgs::IMU>("/imu",10);
  ros::Publisher baro_pub = nh.advertise<alpha_msgs::AirPressure>("/pressure",10);
  //  ros::Publisher ahrs_pub = nh.advertise<geometry_msgs::Quaternion>("/ahrs",10);
  ros::Subscriber calib_sub = nh.subscribe("/calibrate",10,calibCB);
  ros::Publisher pose_pub = nh.advertise<alpha_msgs::FilteredState>("/pose",10);

  AltitudeEstimator altitude_est;
  altitude_est.init();

  MPU9250 imu;
  imu.initialize();

  Ahrs ahrs(100,0.5,0.001);
  int barometer_state = STATE_INITIAL;
  MS5611 barometer;
  barometer.initialize();


  ros::Rate rate(100);
  float accel[3];
  float gyro[3];
  float mag[3];
  float pressure;

  while(ros::ok()){

    updatePressure(barometer_state,barometer,pressure);
    alpha_msgs::AirPressure baro_msg;
    baro_msg.pressure = pressure;
    baro_pub.publish(baro_msg);
    
    imu.getMotion9(&accel[0],&accel[1],&accel[2],
		   &gyro[0],&gyro[1],&gyro[2],
		   &mag[0],&mag[1],&mag[2]);


    mag[0] = (mag[0]-(-14.20312381))*1.000348321;
    mag[1] = (mag[1]-(35.7140657))*0.965438471;
    mag[2] = (mag[2]-(-22.57587933))*1.036753524;
    alpha_msgs::IMU imu_msg;
    imu_msg.linear_acceleration = float2VectorMsg(accel);
    imu_msg.angular_velocity    = float2VectorMsg(gyro);
    imu_msg.magnetic_field      = float2VectorMsg(mag);

    imu_pub.publish(imu_msg);

    
    ahrs.MadgwickAHRSupdate(gyro[0],gyro[1],gyro[2],
			    accel[0],accel[1],accel[2],
                            mag[1],mag[0],-mag[2]);

    

    
    geometry_msgs::Quaternion quat_msg;
    quat_msg.x = (double)ahrs.q1;
    quat_msg.y = (double)ahrs.q2;
    quat_msg.z = (double)ahrs.q3;
    quat_msg.w = (double)ahrs.q0;
    
    float z = altitude_est.update(quat_msg,imu_msg,baro_msg);
    alpha_msgs::FilteredState pose_msg;
    pose_msg.z = z;
    float q0 = quat_msg.w;
    float q1 = quat_msg.x;
    float q2 = quat_msg.y;
    float q3 = quat_msg.z;
    pose_msg.roll = atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
    pose_msg.pitch = asin(2*(q0*q2-q3*q1));
    pose_msg.yaw = add_angle(atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3)),-yaw_offset);
    //the sign might have to be fixed
    if(calibrating){
      calib_count++;
      yaw_sum += pose_msg.yaw;
      if(get_dtime() > calib_start_time + 3000){
	yaw_sum/=calib_count;
	yaw_offset = yaw_sum + M_PI;
	calibrating = false;
      }
    }


    pose_pub.publish(pose_msg);
    
    rate.sleep();
    ros::spinOnce();
    
  }
  return 0;
}
