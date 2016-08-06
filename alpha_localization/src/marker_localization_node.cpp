#include <ros/ros.h>
#include <alpha_localization/MarkerLocalization.h>
#include <alpha_msgs/FilteredState.h>

int main(int argc, char* argv[]){
  ros::init(argc,argv,"marker_localization_node");
  ros::NodeHandle nh;
  ros::Publisher pose_pub = nh.advertise<alpha_msgs::FilteredState>("/marker_pose",10);
  VideoCapture input;
  input.open(0);
  Mat image;

  MarkerLocalization ml;
  ml.loadMarkerPosition();
  ml.loadCameraInfo();

  namedWindow("detection_result");
  startWindowThread();


  while(input.grab() && ros::ok()){
    input.retrieve(image);
    std::vector<Point2f> corners;
    ml.getCVCorners(image,corners);
    if(corners.size() < 4)
      continue;
    
    Pose pose;
    ml.sortCorners(corners);
    for(int i = 0; i < corners.size(); i++)
      std::cout<<corners[i]<<std::endl;

    pose = ml.solvePose(corners);
    alpha_msgs::FilteredState msg;
    msg.x = pose.pos.x;
    msg.y = pose.pos.y;
    msg.z = pose.pos.z;
    msg.roll = pose.rot.x;
    msg.pitch = pose.rot.y;
    msg.yaw = pose.rot.z;
    pose_pub.publish(msg);
    pose.print();
  }
  return 0;

}
