#include "generalController.hpp"
#include <iostream>

void GeneralController::callbackPubSrvRequest(const system_client::MsgRequest &msg)
{
}

void GeneralController::callbackPubSrvRobotData()
{
    auto r = rc.getRobot();
    system_client::MsgRobotData msg;

    msg.id = r->getId();
    msg.currLocation = r->getCurrentLocation();
    msg.minSpeed = r->getMediumVelocity();
    msg.status = r->getStatus();

    ros::Publisher pubSrvRobotData = nh.advertise<system_client::MsgRobotData>(srvRobotDataTopic, 1);
    pubSrvRobotData.publish(msg);
    ros::spinOnce();
}

void GeneralController::callbackSubRequest(const system_client::MsgRequest &msg)
{
    switch (msg.type)
    {
    case system_client::MsgRequest::ROBOT_DATA:
        callbackPubSrvRobotData();
        break;

    case system_client::MsgRequest::CANCEL_TASK:
        break;

    default:
        break;
    }
}

void GeneralController::callbackSubTask(const system_client::MsgTaskList &msg)
{
    for (auto t : msg.taskList)
        tc.push(t);
}

void GeneralController::run()
{
    subRequest = nh.subscribe("myResquestTopic", 100, &GeneralController::callbackSubRequest, this);
    subTask = nh.subscribe("myTaskTopic", 100, &GeneralController::callbackSubTask, this);

    //falta general DAO
    lc.updateLocationList();
    lc.updateDistanceMatrix();

    ros::spin();
}