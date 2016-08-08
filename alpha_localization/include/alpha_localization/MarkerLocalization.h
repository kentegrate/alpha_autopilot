#ifndef ALPHA_LOCALIZATION_MARKER_LOCALIZATION_H
#define ALPHA_LOCALIZATION_MARKER_LOCALIZATION_H

#include <ros/ros.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <alpha_msgs/FilteredState.h>

using namespace cv;

struct Vector3{
  float x;
  float y;
  float z;
  Vector3(){
    x = 0; y = 0; z = 0;
  }
  Vector3(float _x, float _y, float _z){
    x = _x; y = _y; z = _z;
  }
  void print(){
    std::cout<<"x "<<x<<" y "<<y<<" z "<<z<<std::endl;
  }
};
struct Pose{
  Vector3 pos;
  Vector3 rot;
  void print(){
    std::cout<<"position ";
    pos.print();
    std::cout<<"rotation ";
    rot.print();
  }
};
class MarkerLocalization{
 private:
  ros::Subscriber ahrs_sub;
  ros::NodeHandle private_nh;
  std::vector<Point3f> objectPoints;

  
  Mat camMatrix;
  Mat distCoeffs;
  Vector3 ahrs_euler;
  
 public:
  MarkerLocalization();
  void init();
  void ahrsCB(const alpha_msgs::FilteredState::ConstPtr msg);
  void loadMarkerPosition();
  void getCVCorners(Mat &input, std::vector<Point2f> &corners,Mat &debug_image);
  void loadCameraInfo();
  Pose solvePose(std::vector<Point2f> &corners);
  void sortCorners(std::vector<Point2f> &corners);
  void sort_helper(std::vector<Point2f> &sorting, std::pair<float,float> pt1, std::pair<float,float> pt2, bool reverse);



};



#endif //ALPHA_LOCALIZATION_MARKER_LOCALIZATION_H
