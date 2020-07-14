#include "generalController.hpp"
#include <iostream>

void GeneralController::callbackPubSrvRequest(const system_client::MsgRequest &msg)
{

}

void GeneralController::callbackPubSrvRobotData()
{
    /*auto r = rc.getRobot();
    system_client::MsgRobotData msg;

    msg.id = r->getId();
    msg.currLocation = r->getCurrentLocation();
    msg.minSpeed = r->getMediumVelocity();
    msg.status = r->getStatus();*/

}

void GeneralController::callbackSubRequest(const system_client::MsgRequest &msg)
{
    std::cout << (int)msg.type << std::endl;
}

void GeneralController::callbackSubTask(const system_client::MsgTaskList &msg)
{
    std::cout << msg.taskList[0].id << std::endl;
}

void GeneralController::run()
{
    subRequest = nh.subscribe("myResquestTopic", 100, &GeneralController::callbackSubRequest, this);
    subTask = nh.subscribe("myTaskTopic", 100, &GeneralController::callbackSubTask, this);
    
    ros::spin();
}