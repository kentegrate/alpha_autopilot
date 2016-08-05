#include <alpha_simulator/AircraftPlugin.h>
#include <alpha_msgs/FilteredState.h>
namespace gazebo
{
  AircraftPlugin::~AircraftPlugin(){
    this->rosnode_->shutdown();
    delete this->rosnode_;
  }
  
  void AircraftPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){
    if(!ros::isInitialized()){
      int argc = 0;
      char** argv = NULL;
      ros::init(argc,argv,"aircraft_plugin",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
    }
    this->rosnode_ = new ros::NodeHandle;
    //    this->rc_sub = this->rosnode_->subscribe("/rc_out",10,&AircraftPlugin::RCinput,this);
    this->pose_pub = this->rosnode_->advertise<alpha_msgs::FilteredState>("/pose",10);
    this->model = _parent;
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&AircraftPlugin::update,this,_1));

    offset.set(1511,1503,1510,1108);
    last_update = common::Time::GetWallTime();
    update_rate = ros::Duration(0.03);
    last_ros_update = ros::Time::now();
    aircraft_model.initProperties();
    aircraft_model.Previous.Global.q.SetFromEuler(Vector3(0,M_PI,0));
    aircraft_model.Previous.Local.V.x = 1.0;
    aircraft_model.Previous.Global.P.y = 0.1;
  }

  void AircraftPlugin::update(const common::UpdateInfo &/*_info*/){

    double dt = (common::Time::GetWallTime() - last_update).Double();
    //    control.print();
    aircraft_model.update(control,dt);

    aircraft_model.getGZPose(pose);
    pose.pos.x = 0;//for debug
    pose.pos.y = 0;
    this->model->SetWorldPose(pose);
    //    std::cout<<"pose "<<pose<<std::endl;

    last_update = common::Time::GetWallTime();

    if(ros::Time::now() > last_ros_update + update_rate){
      RCinput();
      publishOdometry(pose);
      ros::spinOnce();
      last_ros_update = ros::Time::now();
    }
  }

  void AircraftPlugin::RCinput(){
    std::vector<int> Channel;
    AircraftControl raw_input;
    if(!(this->rosnode_->getParam("/rc_out",Channel))){
      raw_input = offset;
    }
    else{
      raw_input.set(Channel[1],Channel[3],
		    Channel[0],Channel[2]);
    }
    control = raw_input - offset;

    control.calc_magnitude(1915,1095,
			   1915,1095,
			   M_PI/3,-M_PI/3,
			   1924,1108);

  }
  void AircraftPlugin::publishOdometry(gazebo::math::Pose  pose){
    alpha_msgs::FilteredState msg;
    gazebo::math::Vector3 euler = pose.rot.GetAsEuler();
    msg.x = pose.pos.x;
    msg.y = pose.pos.y;
    msg.z = pose.pos.z;

    msg.roll = euler.x;
    msg.pitch = euler.y;
    msg.yaw = euler.z;
    pose_pub.publish(msg);
  }
  GZ_REGISTER_MODEL_PLUGIN(AircraftPlugin)

}
