#include <alpha_simulator/AircraftPlugin.h>

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
    this->rc_sub = this->rosnode_->subscribe("/rc",10,&AircraftPlugin::RCinput,this);
    this->model = _parent;
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&AircraftPlugin::update,this,_1));

    offset.set(1500,1500,1500,1100);
    last_update = common::Time::GetWallTime();
    update_rate = ros::Duration(0.003);
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

    this->model->SetWorldPose(pose);
    //    std::cout<<"pose "<<pose<<std::endl;

    last_update = common::Time::GetWallTime();

    if(ros::Time::now() > last_ros_update + update_rate){

      ros::spinOnce();
      last_ros_update = ros::Time::now();
    }
  }

  void AircraftPlugin::RCinput(const alpha_msgs::RC::ConstPtr msg){

    AircraftControl raw_input(msg->Channel[1],msg->Channel[3],
			      msg->Channel[0],msg->Channel[2]);

    control = raw_input - offset;

    control.calc_magnitude(M_PI/3,-M_PI/3,
			   M_PI/3,-M_PI/3,
			   M_PI/3,-M_PI/3,
			   1,-1);

  }
  GZ_REGISTER_MODEL_PLUGIN(AircraftPlugin)

}
