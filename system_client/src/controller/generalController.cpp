#include "generalController.hpp"
#include <iostream>

GeneralController::GeneralController()
{
    this->nav = Navigator("robot1");

    std::string host = "localhost";
    std::string user = "root";
    std::string pass = "281094";
    std::string db = "ServerDB";

    gd = GeneralDao(host, user, pass, db);
    lc = LocationController(&gd);
    rc = RobotController(&gd, 0);
    tc = TaskController();
}

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

void GeneralController::callbackCancelTask(uint32_t id)
{
    system_client::MsgTask t;
    if (tc.getFirst(t))
    {
        if (id == t.id)
            navPrms.set_value();
        else
            tc.deleteTaskById(id);     
    }
}

void GeneralController::callbackSubRequest(const system_client::MsgRequest &msg)
{
    switch (msg.type)
    {
    case system_client::MsgRequest::ROBOT_DATA:
        callbackPubSrvRobotData();
        break;

    case system_client::MsgRequest::CANCEL_TASK:
        callbackCancelTask(msg.data);
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

void GeneralController::performTask(const system_client::MsgTask t)
{
    ros::Rate r(5);

    //Going to pickup
    auto pickUp = lc.getLocationById(t.pickUp, false);
    nav.navigateTo(0.1, 0.1, 1.3);
    while (nav.stillNavigating())
    {
        if (!(navFtr.wait_for(std::chrono::milliseconds(0)) == std::future_status::timeout))
        {
            nav.cancel();
            tc.deleteTaskById(t.id);
            navPrms = std::promise<void>();
            navFtr = navPrms.get_future();
            return;
        }
        r.sleep();
    }

    //Going to delivery
    auto delivery = lc.getLocationById(t.delivery, false);
    nav.navigateTo(delivery->getX(), delivery->getY(), delivery->getA());
    while (nav.stillNavigating())
    {
        if (!(navFtr.wait_for(std::chrono::milliseconds(0)) == std::future_status::timeout))
        {
            nav.cancel();
            tc.deleteTaskById(t.id);
            navPrms = std::promise<void>();
            navFtr = navPrms.get_future();
            return;
        }
        r.sleep();
    }
}

void GeneralController::performTasks()
{
    navPrms = std::promise<void>();
    navFtr = navPrms.get_future();
    ros::Rate r(5);

    while (true)
    {
        system_client::MsgTask t;
        if (tc.getFirst(t))
            performTask(t);
        r.sleep();
    }
}

void GeneralController::run()
{
    //Get locations and distances
    lc.updateLocationList();
    lc.updateDistanceMatrix();

    //Get robor data from DB
    rc.updateRobotFromDB();

    //Start navigation
    nav.start();

    //Start listeners request and tasks
    subRequest = nh.subscribe("myResquestTopic", 100, &GeneralController::callbackSubRequest, this);
    subTask = nh.subscribe("myTaskTopic", 100, &GeneralController::callbackSubTask, this);

    std::thread prfTks(&GeneralController::performTasks, this);

    ros::spin();
}