#include <ros/ros.h>
#include <algorithm>
#include <opencv2/opencv.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;


int main(int argc, char *argv[]){
  ros::init(argc, argv, "raspi_camera");
  ros::NodeHandle nh;
  
  VideoCapture input;
  input.open(0);
  Mat image;

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/camera/image_raw",1);


  ros::Rate rate(15);
  while(input.grab() && ros::ok()){
    input.retrieve(image);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",image).toImageMsg();
    pub.publish(msg);
    rate.sleep();

  }
  return 0;
}
