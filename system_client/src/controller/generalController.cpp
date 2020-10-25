#include "generalController.hpp"
#include <iostream>
#include <chrono>

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

    stopTask = false;
    goToCharge = false;

    bs = BatterySimulator(100, 1, 0.001);
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
            stopTask = true;
        else
            tc.deleteTaskById(id);
    }
}

void GeneralController::callbackSubRequest(const system_client::MsgRequest &msg)
{
    switch (msg.type)
    {
    case system_client::MsgRequest::ROBOT_CHECK:
        callbackPubSrvRobotData();
        break;

    case system_client::MsgRequest::FREE_ROBOT:
        break;

    case system_client::MsgRequest::CHARGE_BATTERY:
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
    std::cout << "Going to depot" << std::endl;
    rc.getRobot()->setStatus(Robot::STATUS_TO_DEPOT);
    nav.navigateTo(depot->getX(), depot->getY(), depot->getA());

    while (ros::ok() && nav.stillNavigating())
        ; //wait arrive in depot

    if (nav.hasArrived())
    {
        rc.getRobot()->setCurrentLocation(rc.getRobot()->getDepot());
        if (goToCharge)
            rc.getRobot()->setStatus(Robot::STATUS_CHARGING);
        else
            rc.getRobot()->setStatus(Robot::STATUS_FREE);
    }
    else
    {
        rc.getRobot()->setStatus(Robot::STATUS_FAIL);
        callbackPubSrvRobotData();
    }
}

void GeneralController::performTask(const system_client::MsgTask t)
{
    ros::Rate r(5);
    ros::Duration d(3.0);
    system_client::MsgRequest taskStatus;
    taskStatus.data = t.id;
    taskStatus.type = system_client::MsgRequest::PERFORMING_PICK_UP;

    stopTask = false;

    rc.getRobot()->setStatus(Robot::STATUS_WORKING);
    //Going to pickup
    callbackPubSrvRequest(taskStatus);
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
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        nav.navigateTo(pickUp->getX(), pickUp->getY(), pickUp->getA());
        while (ros::ok() && nav.stillNavigating())
        {
            //Used to cancel task in execution
            if (stopTask || goToCharge)
            {
                nav.cancel();
                return;
            }
            r.sleep();
            ros::spinOnce();
        }
        if (!nav.hasArrived())
        {
            taskStatus.type = system_client::MsgRequest::FAIL_TASK;
            taskStatus.data = t.id;
            nav.cancel();
            callbackPubSrvRequest(taskStatus);
            return;
        }

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / 1000.0;
        double distance = lc.getDistance(rc.getRobot()->getCurrentLocation(), pickUp->getId());
        rc.getRobot()->updateMediumVelocity(distance, time);

        //Set current location
        rc.getRobot()->setCurrentLocation(pickUp->getId());
        d.sleep();
        ros::spinOnce();

        //Turn around 180 degres
        std::cout << "Turn around 180 degres" << std::endl;
        nav.navigateTo(pickUp->getX(), pickUp->getY(), -pickUp->getA());
        while (ros::ok() && nav.stillNavigating())
        {
            //Used to cancel task in execution
            if (stopTask || goToCharge)
            {
                nav.cancel();
                return;
            }
            r.sleep();
            ros::spinOnce();
        }
        if (!nav.hasArrived())
        {
            taskStatus.type = system_client::MsgRequest::FAIL_TASK;
            taskStatus.data = t.id;
            nav.cancel();
            callbackPubSrvRequest(taskStatus);
            return;
        }
    }

    //Going to delivery
    taskStatus.type = system_client::MsgRequest::PERFORMING_DELIVERY;
    callbackPubSrvRequest(taskStatus);
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
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        nav.navigateTo(delivery->getX(), delivery->getY(), delivery->getA());
        while (ros::ok() && nav.stillNavigating())
        {
            if (stopTask || goToCharge)
            {
                nav.cancel();
                return;
            }
            r.sleep();
            ros::spinOnce();
        }
        if (!nav.hasArrived())
        {
            taskStatus.type = system_client::MsgRequest::FAIL_TASK;
            taskStatus.data = t.id;
            nav.cancel();
            callbackPubSrvRequest(taskStatus);
            return;
        }

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / 1000.0;
        double distance = lc.getDistance(rc.getRobot()->getCurrentLocation(), delivery->getId());
        rc.getRobot()->updateMediumVelocity(distance, time);

        //Set current location
        rc.getRobot()->setCurrentLocation(delivery->getId());
        d.sleep();
        ros::spinOnce();

        //Turn around 180 degres
        std::cout << "Turn around 180 degres" << std::endl;
        nav.navigateTo(delivery->getX(), delivery->getY(), -delivery->getA());
        while (ros::ok() && nav.stillNavigating())
        {
            //Used to cancel task in execution
            if (stopTask || goToCharge)
            {
                nav.cancel();
                return;
            }
            r.sleep();
            ros::spinOnce();
        }
        if (!nav.hasArrived())
        {
            taskStatus.type = system_client::MsgRequest::FAIL_TASK;
            taskStatus.data = t.id;
            nav.cancel();
            callbackPubSrvRequest(taskStatus);
            return;
        }
    }
    taskStatus.type = system_client::MsgRequest::SUCESS_TASK;
    taskStatus.data = t.id;
    callbackPubSrvRequest(taskStatus);
}

void GeneralController::performTasks()
{
    ros::Rate r(5);
    bool inDepot = true;

    while (ros::ok())
    {
        system_client::MsgTask t;
        if (tc.getFirst(t) && !goToCharge)
        {
            inDepot = false;
            performTask(t);
            tc.pop();
        }
        else if (!inDepot) //Case performed all tasks and it is not in the depot, go to it
        {
            inDepot = true;

            goToDepot();

            while (ros::ok() && goToCharge) //Waiting untill fully charged
                r.sleep();
        }

        r.sleep();
    }
}

void GeneralController::callbackBattery()
{
    ros::Rate r(1);

    while (ros::ok())
    {
        rc.getRobot()->setRemainingBattery(bs.getRemaningBattery());
        if (rc.getRobot()->getUtilRemainingBattery() <= 0)
        {
            goToCharge = true;
            //Notificar cancelamento das tarefas
            system_client::MsgTask tsk;
            system_client::MsgRequest msg;
            msg.type = system_client::MsgRequest::FAIL_TASK;
            while (tc.getFirst(tsk))
            {
                msg.data = tsk.id;
                callbackPubSrvRequest(msg);
                tc.pop();
            }
            //Carregar bateria quando chegar no deposityo
        }
        auto rbt = rc.getRobot();
        std::cout << *rbt << std::endl;
        r.sleep();
        ros::spinOnce();
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

    //Waiting topiscs start
    ros::Duration d(1);
    d.sleep();
    ros::spinOnce();

    std::thread thrTasks(&GeneralController::performTasks, this);
    std::thread thrBattery(&GeneralController::callbackBattery, this);
    /*stopTask = true;
    goToCharge = true;
    //Notificar falha de todas
    tc.clear();
    d.sleep();
    d.sleep();
    goToCharge = false;
    tc.push(t);*/
    ros::spin();
}