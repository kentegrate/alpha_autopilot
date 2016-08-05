#include <ros/ros.h>

#include <alpha_msgs/IMU.h>
#include <alpha_msgs/AirPressure.h>

#include <alpha_drivers/spi_sensors_nodelet.h>
#include <pluginlib/class_list_macros.h>



namespace alpha_autopilot{
  void SPISensors::onInit(){
    ros::NodeHandle &nh = getNodeHandle();
    imu_pub = nh.advertise<alpha_msgs::IMU>("/imu",10);
    baro_pub = nh.advertise<alpha_msgs::AirPressure>("/pressure",10);
    imu.initialize();
    barometer.initialize();
    
    event_timer = nh.createTimer(ros::Duration(0.01),&SPISensors::updateSensors,this);
  }
  float SPISensors::get_ftime()
  {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    //ミリ秒を計算
    return ((float)(tv.tv_sec)*1000 + (float)(tv.tv_usec)*0.001); //★
  }

  void SPISensors::updatePressure(){
    switch(state){
    case STATE_INITIAL:
      barometer.refreshPressure();
      last_baro_time = get_ftime();
      state = STATE_WAIT_PRESSURE;
      break;
    case STATE_WAIT_PRESSURE:
      if(get_ftime()-last_baro_time > 10){
	barometer.readPressure();
	barometer.refreshTemperature();
	state = STATE_WAIT_TEMPERATURE;
	last_baro_time = get_ftime();
      }
      break;
    case STATE_WAIT_TEMPERATURE:
      if(get_ftime()-last_baro_time > 10){
	barometer.readTemperature();
	barometer.calculatePressureAndTemperature();
	barometer.refreshPressure();
	last_baro_time = get_ftime();
	state = STATE_WAIT_PRESSURE;
      }
      break;
    }
    void barometer.getPressure();
  }
  void SPISensors::updateIMU(){
    imu.getMotion9(&accel[0],&accel[1],&accel[2],
		   &gyro[0],&gyro[1],&gyro[2],
		   &mag[0],&mag[1],&mag[2]);
  }
  void SPISensors::updateSensors(const ros::TimerEvent &msg){
    float old_pressure = pressure;
    updatePressure();
    if(old_pressure != pressure)
      publish_baro();

    updateIMU();
    publish_imu();

  }
  void SPISensors::publish_baro(){
    alpha_msgs::AirPressurePtr msg(new alpha_msgs::AirPressure);
    msg->pressure = pressure;
    baro_pub.publish(msg);
  }
  void SPISensors::publish_imu(){
    alpha_msgs::IMUPtr msg(new alpha_msgs::IMU);
    msg->linear_acceleration.x = accel[0];
    msg->linear_acceleration.y = accel[1];    
    msg->linear_acceleration.z = accel[2];

    msg->angular_velocity.x = gyro[0];
    msg->angular_velocity.y = gyro[1];    
    msg->angular_velocity.z = gyro[2];

    msg->magnetic_field.x = mag[0];
    msg->magnetic_field.y = mag[1];    
    msg->magnetic_field.z = mag[2];
    imu_pub.publish(msg);
  }

  PLUGINLIB_DECLARE_CLASS(alpha_autopilot,SPISensors,alpha_autopilot::SPISensors, nodelet::Nodelet);
}
