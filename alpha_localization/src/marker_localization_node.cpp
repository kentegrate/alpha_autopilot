#include <ros/ros.h>
#include <alpha_localization/MarkerLocalization.h>

int main(int argc, char* argv[]){
  ros::init(argc,argv,"marker_localization_node");
  ros::NodeHandle nh;
  
  VideoCapture input;
  input.open(0);
  Mat image;

  MarkerLocalization ml;
  ml.loadMarkerPosition();
  ml.loadCameraInfo();


  while(input.grab() && ros::ok()){
    input.retrieve(image);
    std::vector<Point2f> corners;
    ml.getCVCorners(image,corners);
    if(corners.size() < 4)
      continue;
    
    Pose pose;
    pose = ml.solvePose(corners);
    pose.print();
  }
  return 0;

}
