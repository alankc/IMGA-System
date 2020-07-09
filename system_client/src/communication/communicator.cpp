#include "communicator.hpp"

Communicator::Communicator(/* args */)
{
}

Communicator::~Communicator()
{
    thrRequest->detach();
    thrTask->detach();
    delete(thrRequest);
    delete(thrTask);
}

void Communicator::listnerRequest()
{
    ros::Rate r(1);
    while (1)
    {
        ROS_INFO("listnerRequest");
        r.sleep();
    }
        
}

void Communicator::callbackRequest()
{
}

void Communicator::listnerTask()
{
    ros::Rate r(1);
    while (1)
    {
        ROS_INFO("listnerTask");
        r.sleep();
    }
        
}

void Communicator::callbackTask()
{
}

void Communicator::sendRobotData()
{
}

void Communicator::run()
{
    thrRequest = new std::thread(&Communicator::listnerRequest, this);
    thrTask = new std::thread(&Communicator::listnerTask, this);
}