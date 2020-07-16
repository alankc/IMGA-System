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

void GeneralController::performTask(const system_client::MsgTask &t)
{
    std::cout << "\n\n"
              << t.deadline << std::endl;
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(5000));
}

void GeneralController::performTasks()
{
    system_client::MsgTask t;
    t.deadline = 16.9;
    //while (!navMtx.try_lock());

    //while (tc.getFirst(t))
    //{
    //navega para pick up
    //espera navegação
    //navThr = new std::thread();

    auto future = std::async(std::launch::async, &GeneralController::performTask, this, std::ref(t));
    auto status = future.wait_for(std::chrono::seconds(3));
    std::promise<void> signal_exit;
    while (status != std::future_status::timeout)
    {
        status = future.wait_for(std::chrono::duration<double, std::milli>(1000));
    }
    std::cout << "\n\n ready" << std::endl;
    //}
}

void GeneralController::run()
{
    /*subRequest = nh.subscribe("myResquestTopic", 100, &GeneralController::callbackSubRequest, this);
    subTask = nh.subscribe("myTaskTopic", 100, &GeneralController::callbackSubTask, this);

    //falta general DAO
    lc.updateLocationList();
    lc.updateDistanceMatrix();*/
    performTasks();

    ros::spin();
}