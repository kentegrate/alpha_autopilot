
#include <ros/ros.h>
#include <alpha_drivers/ms5611.h>
#include <alpha_drivers/mpu9250.h>

#include <alpha_drivers/IMU.h>
#include <alpha_drivers/AirPressure.h>
#include <geometry_msgs/Vector3.h>

geometry_msgs::Vector3 float2VectorMsg(float* data){
  geometry_msgs msg;
  msg.x = (double)data[0];
  msg.y = (double)data[1];
  msg.z = (double)data[2];
  return msg;
}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "spi_sensor_node");
  ros::NodeHandle nh;

  ros::Publisher imu_pub = nh.advertise<alpha_drivers::IMU>("/imu",10);
  ros::Publisher baro_pub = nh.advertise<alpha_drivers::AirPressure>("/pressure",10);
  
  MPU9250 imu;
  imu.initialize();

  MS5611 barometer;
  ms5611.initialize();

  ros::Rate rate(30);
  float accel[3];
  float gyro[3];
  float mag[3];

  while(ros::ok()){
    mpu9250.getMotion9(&accel[0],&accel[1],&accel[2],
		       &gyro[0],&gyro[1],&gyro[2],
		       &mag[0],&mag[1],&mag[2]);
    alpha_drivers::IMU imu_msg;
    imu_msg.linear_acceleration = float2VectorMsg(accel);
    imu_msg.angular_velocity    = float2VectorMsg(gyro);
    imu_msg.magnetic_field      = float2VectorMsg(mag);

    imu_pub.publish(imu_msg);

    barometer.update();
    alpha_drivers::AirPressure baro_msg;
    baro_msg.pressure = (float)ms5611.getPressure();
    baro_pub.publish(baro_msg);

  }
  return 0;
}
