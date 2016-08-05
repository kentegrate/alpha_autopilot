#ifndef ALPHA_DRIVERS_DECODER_NODELET_H
#define ALPHA_DRIVERS_DECODER_NODELET_H

#include <ros/ros.h>
#include <alpha_drivers/decoder/GPIO_RPI.h>
#include <alpha_drivers/decoder/RCInput_RPI.h>
#include <alpha_msgs/RC.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
namespace alpha_autopilot{
  class SBUSDecoder : public nodelet::Nodelet{
  private:
    ros::Publisher rc_pub;
    GPIO_RPI gpio;
    RCInput_RPI rcin;
    ros::Timer event_timer;
  public:
    SBUSDecoder(){
      rcin.set_gpio(&gpio);
    }
    ~SBUSDecoder(){
      rcin.deinit();
    }
    virtual void onInit();
    void timerEvent(const ros::TimerEvent &msg);
  };
}

#endif //ALPHA_DRIVERS_DECODER_NODELET_H
