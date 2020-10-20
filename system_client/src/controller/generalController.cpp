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
    srvRequestTopic = "rqt123";
    srvRobotDataTopic = "robdata";
}

void GeneralController::callbackPubSrvRequest(const system_client::MsgRequest &msg)
{
    pubSrvRequest.publish(msg);
    ros::spinOnce();
}

void GeneralController::callbackPubSrvRobotData()
{
    auto r = rc.getRobot();
    system_client::MsgRobotData msg;

    msg.id = r->getId();
    msg.currLocation = r->getCurrentLocation();
    msg.minSpeed = r->getMediumVelocity();
    msg.status = r->getStatus();

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
    nav.cancel();
    for (auto t : msg.taskList)
        tc.push(t);
}

void GeneralController::goToDepot()
{
    auto depot = lc.getLocationById(rc.getRobot()->getDepot(), true);
    if (depot == NULL)
    {
        lc.updateLocationList();
        lc.updateDistanceMatrix();
        depot = lc.getLocationById(rc.getRobot()->getDepot(), true);
    }
    nav.navigateTo(depot->getX(), depot->getY(), depot->getA());
}

void GeneralController::performTask(const system_client::MsgTask t)
{
    ros::Rate r(5);
    ros::Duration d(3.0);

    //Going to pickup
    std::cout << "Going to pickup" << std::endl;
    auto pickUp = lc.getLocationById(t.pickUp, false);
    if (pickUp == NULL)
    {
        lc.updateLocationList();
        lc.updateDistanceMatrix();
        pickUp = lc.getLocationById(t.pickUp, false);
    }

    if (rc.getRobot()->getCurrentLocation() != pickUp->getId())
    {
        nav.navigateTo(pickUp->getX(), pickUp->getY(), pickUp->getA());
        while (nav.stillNavigating())
        {
            //Used to cancel task in execution
            if (!(navFtr.wait_for(std::chrono::milliseconds(0)) == std::future_status::timeout))
            {
                nav.cancel();
                navPrms = std::promise<void>();
                navFtr = navPrms.get_future();
                return;
            }
            r.sleep();
        }
        if (!nav.hasArrived())
        {
            system_client::MsgRequest m;
            m.type = system_client::MsgRequest::FAIL_TASK;
            m.data = t.id;
            nav.cancel();
            callbackPubSrvRequest(m);
            return;
        }

        //Set current location
        rc.getRobot()->setCurrentLocation(pickUp->getId());
        d.sleep();
        ros::spinOnce();

        //Turn around 180 degres
        std::cout << "Turn around 180 degres" << std::endl;
        nav.navigateTo(pickUp->getX(), pickUp->getY(), -pickUp->getA());
        while (nav.stillNavigating())
        {
            //Used to cancel task in execution
            if (!(navFtr.wait_for(std::chrono::milliseconds(0)) == std::future_status::timeout))
            {
                nav.cancel();
                navPrms = std::promise<void>();
                navFtr = navPrms.get_future();
                return;
            }
            r.sleep();
        }
        if (!nav.hasArrived())
        {
            system_client::MsgRequest m;
            m.type = system_client::MsgRequest::FAIL_TASK;
            m.data = t.id;
            nav.cancel();
            callbackPubSrvRequest(m);
            return;
        }
    }

    //Going to delivery
    std::cout << "Going to delivery" << std::endl;
    auto delivery = lc.getLocationById(t.delivery, false);
    if (delivery == NULL)
    {
        lc.updateLocationList();
        lc.updateDistanceMatrix();
        delivery = lc.getLocationById(t.delivery, false);
    }

    if (rc.getRobot()->getCurrentLocation() != delivery->getId())
    {
        nav.navigateTo(delivery->getX(), delivery->getY(), delivery->getA());
        while (nav.stillNavigating())
        {
            if (!(navFtr.wait_for(std::chrono::milliseconds(0)) == std::future_status::timeout))
            {
                nav.cancel();
                navPrms = std::promise<void>();
                navFtr = navPrms.get_future();
                return;
            }
            r.sleep();
        }
        if (!nav.hasArrived())
        {
            system_client::MsgRequest m;
            m.type = system_client::MsgRequest::FAIL_TASK;
            m.data = t.id;
            nav.cancel();
            callbackPubSrvRequest(m);
            return;
        }

        //Set current location
        rc.getRobot()->setCurrentLocation(delivery->getId());
        d.sleep();
        ros::spinOnce();

        //Turn around 180 degres
        std::cout << "Turn around 180 degres" << std::endl;
        nav.navigateTo(delivery->getX(), delivery->getY(), -delivery->getA());
        while (nav.stillNavigating())
        {
            //Used to cancel task in execution
            if (!(navFtr.wait_for(std::chrono::milliseconds(0)) == std::future_status::timeout))
            {
                nav.cancel();
                navPrms = std::promise<void>();
                navFtr = navPrms.get_future();
                return;
            }
            r.sleep();
        }
        if (!nav.hasArrived())
        {
            system_client::MsgRequest m;
            m.type = system_client::MsgRequest::FAIL_TASK;
            m.data = t.id;
            nav.cancel();
            callbackPubSrvRequest(m);
            return;
        }
    }
    system_client::MsgRequest m;
    m.type = system_client::MsgRequest::SUCESS_TASK;
    m.data = t.id;
    callbackPubSrvRequest(m);
}

void GeneralController::performTasks()
{
    navPrms = std::promise<void>();
    navFtr = navPrms.get_future();
    ros::Rate r(5);
    bool inDepot = true;

    while (true)
    {
        system_client::MsgTask t;
        if (tc.getFirst(t))
        {
            inDepot = false;
            performTask(t);
            tc.pop();
        }
        else if (!inDepot) //Case performed all tasks and it is not in the depot, go to it
        {
            inDepot = true;

            //send msgs free
            std::cout << "Going to depot" << std::endl;
            goToDepot();
        }

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

    system_client::MsgTask t;
    t.id = 0;
    t.deadline = 100;
    t.pickUp = 1;
    t.delivery = 2;
    tc.push(t);
    t.id = 1;
    t.deadline = 200;
    t.pickUp = 2;
    t.delivery = 3;
    tc.push(t);

    //Start listeners request and tasks
    subRequest = nh.subscribe("myResquestTopic", 10, &GeneralController::callbackSubRequest, this);
    subTask = nh.subscribe("myTaskTopic", 10, &GeneralController::callbackSubTask, this);

    pubSrvRobotData = nh.advertise<system_client::MsgRobotData>(srvRobotDataTopic, 10);
    pubSrvRequest = nh.advertise<system_client::MsgRequest>(srvRequestTopic, 10);

    std::thread prfTks(&GeneralController::performTasks, this);

    ros::spin();
}