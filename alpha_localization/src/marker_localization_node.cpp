#include <ros/ros.h>
#include <alpha_localization/MarkerLocalization.h>
#include <alpha_msgs/FilteredState.h>
#include <signal.h>
#include <alpha_drivers/RaspiCam.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <time.h>
bool running = true;
RaspiCam input;
float k_q = 0.2;
float k_t = 0.2;
void quaternion_to_euler(float* q,float* euler){
  euler[0] = atan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2]));
  euler[1] = asin(2*(q[0]*q[2]-q[3]*q[1]));
  euler[2] = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
}
void euler_to_quaternion(float* euler, float* q){
  float c1 = cos(euler[0]/2);
  float c2 = cos(euler[1]/2);
  float c3 = cos(euler[2]/2);
  float s1 = sin(euler[0]/2);
  float s2 = sin(euler[1]/2);
  float s3 = sin(euler[2]/2);

  q[0] = c1*c2*c3 - s1*s2*s3;
  q[1] = s1*s2*c3 + c1*c2*s3;
  q[2] = s1*c2*c3 + c1*s2*s3;
  q[3] = c1*s2*c3 - s1*c2*s3;
}
void termination_handler(int sig){
  running = false;
  //  input.releaseCamera();
  ros::shutdown();
}
double get_dtime(void)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  //ミリ秒を計算
  return ((double)(tv.tv_sec)*1000 + (double)(tv.tv_usec)*0.001); //★
}
int main(int argc, char* argv[]){
  ros::init(argc,argv,"marker_localization_node",ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ros::Publisher pose_pub = nh.advertise<alpha_msgs::FilteredState>("/marker_pose",10);
  image_transport::ImageTransport it(nh);
  image_transport::Publisher image_pub = it.advertise("/camera/image_raw",1);


  for(int i = 0; i < 64; i++){
    struct sigaction sa;
    memset(&sa,0,sizeof(sa));
    sa.sa_handler = termination_handler;
    sigaction(i,&sa,NULL);
  }


  input.init(640,480,15,true);
  Mat image;
  

  MarkerLocalization ml;
  ml.init();
  ml.loadMarkerPosition();
  ml.loadCameraInfo();

  //  namedWindow("detection_result");
  //  startWindowThread();

  std::vector<float> q(4,0);
  q[0] = 1;
  std::vector<float> trans(3,0);

  while(running){

    input.getFrame(image);

    double last_update = get_dtime();
    std::vector<Point2f> corners;
    Mat debug_image;
    ml.getCVCorners(image,corners,debug_image);
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(),"mono8",debug_image).toImageMsg();
    image_pub.publish(image_msg);
    if(corners.size() < 4)
      continue;
    
    Pose pose;
    ml.sortCorners(corners);


    pose = ml.solvePose(corners);

    std::vector<float> new_q(4,0);
    std::vector<float> new_euler(3,0);
    new_euler[0] =pose.rot.x;
    new_euler[1] =pose.rot.y;
    new_euler[2] =pose.rot.z;
    std::vector<float> new_trans(3,0);
    new_trans[0] =pose.pos.x;
    new_trans[1] =pose.pos.y;
    new_trans[2] =pose.pos.z+3.05;

    //if(!(new_trans[0] < 0 && new_trans[0] > -40 &&
    //	 new_trans[2] < 10 && new_trans[2] > -2))
    //	 continue;
    euler_to_quaternion(&new_euler[0],&new_q[0]);
    float q_sum = 0;
    for(int i = 0; i < 4; i++){
      q[i] = (1-k_q)*q[i]+k_q*new_q[i];
      q_sum+=q[i]*q[i];
    }
    for(int i = 0; i < 3; i++){
      trans[i] = (1-k_t)*trans[i]+ k_t*new_trans[i];
    }
    float q_norm = sqrt(q_sum);
    for(int i = 0; i <4; i++)
      q[i]/=q_norm;
    std::vector<float> euler(3,0);
    quaternion_to_euler(&q[0],&euler[0]);

    alpha_msgs::FilteredState msg;
    msg.x = trans[0];
    msg.y = trans[1];
    msg.z = trans[2];
    msg.roll = euler[0];
    msg.pitch = euler[1];
    msg.yaw = euler[2];
    pose_pub.publish(msg);
    ros::spinOnce();
    //    pose.print();

  }


  return 0;

}
