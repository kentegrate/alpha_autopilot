#ifndef THREAD_ROS_H
#define THREAD_ROS_H

#include <qthread.h>
#include <iostream>
#include <sys/time.h>

#include <ros/ros.h>
#include "../shared_memory.h"

class Thread_ROS:public QThread
{
public:
    Thread_ROS(Shared_Memory *share_memory);
    ~Thread_ROS();

private:
    Shared_Memory* share_memory;
    ros::Publisher rc_override_pub;

    int RC_Param(std::string s, int i);
    void updateMode();

    bool stop;

protected:
    void run();
};

#endif // THREAD_ROS_H
