
#include <ros/ros.h>
#include <alpha_drivers/ms5611.h>
#include <alpha_drivers/mpu9250.h>
#include <alpha_drivers/ahrs.h>
#include <alpha_drivers/altitude_estimator.h>

#include <alpha_msgs/IMU.h>
#include <alpha_msgs/AirPressure.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#define STATE_INITIAL 1
#define STATE_WAIT_PRESSURE 2
#define STATE_WAIT_TEMPERATURE 3
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
  ros::Publisher pose_pub = nh.advertise<alpha_msgs::FilteredState>("/pose",10);

  AltitudeEstimator altitude_est;
  altitude.init();

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
    
    
    rate.sleep();

  }
  return 0;
}
