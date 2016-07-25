#include <alpha_simulator/SensorRemapper.h>

void SensorRemapper::init(){
  imu_pub = nh.advertise<alpha_msgs::IMU>("/imu",10);
  baro_pub= nh.advertise<alpha_msgs::AirPressure>("/pressure",10);
  
  imu_sub = nh.subscribe("/gazebo/imu",10,&SensorRemapper::imu_CB,this);
  mag_sub = nh.subscribe("/gazebo/magnetic",10,&SensorRemapper::mag_CB,this);
  baro_sub= nh.subscribe("/gazebo/pressure_height",10,&SensorRemapper::baro_CB,this);


}

void SensorRemapper::imu_CB(sensor_msgs::Imu::ConstPtr msg){
  imu_raw = *msg;
}
void SensorRemapper::mag_CB(geometry_msgs::Vector3Stamped::ConstPtr msg){
  mag_raw = msg->vector;
}
void SensorRemapper::baro_CB(geometry_msgs::PointStamped::ConstPtr msg){
  baro_raw = msg->point.z;
}

void SensorRemapper::publish_imu(){
  alpha_msgs::IMU msg;
  msg.angular_velocity = imu_raw.angular_velocity;
  msg.linear_acceleration = imu_raw.linear_acceleration;
  msg.magnetic_field = mag_raw;

  imu_pub.publish(msg);
}
void SensorRemapper::publish_baro(){

  double P0 = 1013.25;//h=0mにおける気圧
  double t0 = 20;//h=0mにおける気温
  double pressure = P0*pow((1-0.0065*baro_raw/(t0+273.2)),5.258);

  alpha_msgs::AirPressure msg;
  msg.pressure = pressure;

  baro_pub.publish(msg);
}

int main(int argc, char* argv[]){

  ros::init(argc,argv,"sensor_remapper");
  ros::NodeHandle nh;

  ros::Rate rate(100);

  SensorRemapper sr;
  sr.init();
  while(ros::ok()){
    
    sr.publish_imu();
    sr.publish_baro();

    ros::spinOnce();
    rate.sleep();
  }




}
