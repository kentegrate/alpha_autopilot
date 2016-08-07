#include <ros/ros.h>
#include <algorithm>
#include <opencv2/opencv.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "RaspiCamCV.h"
using namespace cv;


int main(int argc, char *argv[]){
  ros::init(argc, argv, "raspi_camera");
  ros::NodeHandle nh;
  RASPIVID_CONFIG * config = (RASPIVID_CONFIG*)malloc(sizeof(RASPIVID_CONFIG));

  config->width=640;
  config->height=480;
  config->bitrate=0;// zero: leave as default
  config->framerate=15;
  config->monochrome=1;

  RaspiCamCvCapture * capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture2(0, config);
  free(config);

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/camera/image_raw",1);


  ros::Rate rate(15);
  while(ros::ok()){
    Mat image(raspiCamCvQueryFrame(capture));
    
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"mono8",image).toImageMsg();
    pub.publish(msg);
    rate.sleep();
  }
  raspiCamCvReleaseCapture(&capture);

  return 0;
}
