#include <ros/ros.h>
#include <alpha_drivers/mpu9250.h>

void print_array(std::string msg, float* array, size_t bytes){
  std::cout<<msg<<" ";
  for(int i = 0; i < bytes; i++)
    std::cout<<array[i]<<",";
  std::cout<<std::endl;
}
int main(int argc, char* argv[]){
  ros::init(argc,argv,"test_mpu9250");
  ros::NodeHandle n;
  MPU9250 mpu9250;
  mpu9250.initialize();
  float accel[3];
  float gyro[3];
  float mag[3];
    std::cout<<mpu9250.testConnection()<<std::endl;  
  ros::Rate rate(30);
  while(ros::ok()){
    mpu9250.getMotion9(&accel[0],&accel[1],&accel[2],
		       &gyro[0],&gyro[1],&gyro[2],
		       &mag[0],&mag[1],&mag[2]);

    //    std::cout<<mpu9250.AK8963_whoami()<<std::endl;
      print_array("accel",accel,3);
      /*    print_array("gyro",gyro,3);
    print_array("mag",mag,3);*/
    //    rate.sleep();
  }
  return 0;
}
