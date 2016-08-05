#ifndef ALPHA_DRIVERS_RCOUT_H
#define ALPHA_DRIVERS_RCOUT_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <alpha_drivers/PCA9685.h>
#include <alpha_msgs/RC.h>



namespace alpha_autopilot{
  class RCOut : public nodelet::Nodelet{
  private:
    ros::Subscriber sub;
    virtual void onInit();
    ~RCOut();
    void rc_sub(const alpha_msgs::RC::ConstPtr msg);
    void set_pulse(int16_t *pulse);
    Ada_ServoDriver pwm;

  };
}

#endif //ALPHA_DRIVERS_RCOUT_H
