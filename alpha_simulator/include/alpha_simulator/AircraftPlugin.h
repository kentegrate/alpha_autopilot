#ifndef AIRCRAFT_PLUGIN_H
#define AIRCRAFT_PLUGIN_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <alpha_msgs/RC.h>
#include <alpha_simulator/control.h>
#include <alpha_simulator/AircraftModel.h>
#include <stdio.h>
#include <ros/ros.h>


namespace gazebo{
  class AircraftPlugin : public ModelPlugin
  {
  public:
    virtual ~AircraftPlugin();

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  protected:
    virtual void update(const common::UpdateInfo &/*_info*/);
    virtual void RCinput();
    virtual void publishOdometry(gazebo::math::Pose pose);
  private:
    ros::NodeHandle* rosnode_;
    ros::Subscriber rc_sub;
    ros::Publisher pose_pub;
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;

    AircraftControl control;
    AircraftControl offset;
    AircraftModel aircraft_model;
    gazebo::math::Pose pose;

    ros::Duration update_rate;
    ros::Time last_ros_update;

    common::Time last_update;
  };

}

#endif
