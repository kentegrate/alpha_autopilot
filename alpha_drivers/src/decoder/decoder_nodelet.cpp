#include <alpha_drivers/decoder/decoder_nodelet.h>
#include <pluginlib/class_list_macros.h>
namespace alpha_autopilot{
  SBUSDecoder::onInit(){
    ros::NodeHandle &nh = getNodeHandle();
    rc_pub = nh.advertise<alhpa_msgs::RC>("/rc_in",1);
    gpio.init();
    rcin.init();
    event_timer = nh.createTimer(ros::Duration(0.002),&SBUSDecoder::timerEvent,this);
  }
  SBUSDecoder::timerEvent(const ros::TimerEvent &msg){
    rcin._timer_tick();
    if(rcin.new_input()){
      alpha_msgs::RCPtr msg(new alpha_msgs::RC);
      for(int i = 0; i < LINUX_RC_INPUT_NUM_CHANNELS; i++)
	msg->Channel.push_back(rcin.read(i));
      rc_pub.publish(msg);
    }
  }
  PLUGINLIB_DECLARE_CLASS(alpha_autopilot,SBUSDecoder,alpha_autopilot::SBUSDecoder, nodelet::Nodelet);

}

