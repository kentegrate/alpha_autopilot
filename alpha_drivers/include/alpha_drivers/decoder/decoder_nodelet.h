#include <ros/ros.h>
#include <alpha_drivers/decoder/GPIO_RPI.h>
#include <alpha_drivers/decoder/RCInput_RPI.h>
#include <alpha_msgs/RC.h>
#include <ros/ros.h>

namespace alpha_autopilot{
  class SBUSDecoder : public nodelet::Nodelet{
  private:
    ros::Publisher rc_pub;
    GPIO_RPI gpio;
    RCInput_RPI;
    ros::Timer event_timer;
  public:
    SBUSDecoder(){
      rcin.set_gpio(&gpio);
    }
    ~SUBSDecoder(){
      rcin.deinit();
    }
    virtual void onInit();
    void timerEvent();
  };
}
