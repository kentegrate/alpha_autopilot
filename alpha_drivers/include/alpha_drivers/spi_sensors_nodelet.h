#ifndef ALPHA_DRIVERS_SPI_SENSORS_NODELET_H
#define ALPHA_DRIVERS_SPI_SENSORS_NODELET_H

#include <alpha_drivers/ms5611.h>
#include <alpha_drivers/mpu9250.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#define BARO_STATE_INITIAL 1
#define BARO_STATE_WAIT_PRESSURE 2
#define BARO_STATE_WAIT_TEMPERATURE 3


namespace alpha_autopilot{
  class SPISensors : public nodelet::Nodelet{
    double get_dtime();

    virtual void onInit();
    double last_baro_time;
    void updatePressure();
    void updateIMU();
    void updateSensors(const ros::TimerEvent &msg);
    void publish_baro();
    void publish_imu();
    
    ros::Timer event_timer;
    ros::Publisher baro_pub;
    ros::Publisher imu_pub;    
    MPU9250 imu;
    float accel[3];
    float gyro[3];
    float mag[3];

    MS5611 barometer;
    int barometer_state;
    float pressure;
  };

}
#endif //ALPHA_DRIVERS_SPI_SENSORS_NODELET_H
