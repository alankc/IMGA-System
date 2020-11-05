#include "generalController.hpp"
#include "../scheduler/island.hpp"
#include <chrono>

GeneralController::GeneralController()
{
    //initialize tt, if does not the time is negative...
    tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    //convert tt to broken time
    bt = *std::localtime(&tt);
    //set broken time to the beggining of the day, only way to set tt
    bt.tm_hour = 0;
    bt.tm_min = 0;
    bt.tm_sec = 0;
    //convert broken time back into std::time_t
    tt = std::mktime(&bt);
    // start of today in system_clock units
    zero = std::chrono::system_clock::from_time_t(tt);

    mtx = new std::mutex();
}

GeneralController::GeneralController(std::string host, std::string user, std::string pass, std::string database)
{
    this->gdao = GeneralDao(host, user, pass, database);
    this->rc = RobotController(&gdao);
    this->tc = TaskController(&gdao);
    this->lc = LocationController(&gdao);
    this->sDao = SettingsDao(&gdao);

    //initialize tt, if does not the time is negative...
    tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    //convert tt to broken time
    bt = *std::localtime(&tt);
    //set broken time to the beggining of the day, only way to set tt
    bt.tm_hour = 0;
    bt.tm_min = 0;
    bt.tm_sec = 0;
    //convert broken time back into std::time_t
    tt = std::mktime(&bt);
    // start of today in system_clock units
    zero = std::chrono::system_clock::from_time_t(tt);
    mtx = new std::mutex();
}

GeneralController::~GeneralController() 
{
    delete(mtx);
}

double GeneralController::getCurrentTime_ms()
{
    auto now = std::chrono::system_clock::now();
    auto lengthOfDay = now - zero;
    return std::chrono::duration_cast<std::chrono::milliseconds>(lengthOfDay).count();
}

double GeneralController::getCurrentTime_s()
{
    auto now = std::chrono::system_clock::now();
    auto lengthOfDay = now - zero;
    return std::chrono::duration_cast<std::chrono::seconds>(lengthOfDay).count();
}

void GeneralController::callScheduler()
{
    rc.updateFreeRobots();
    lc.updateLocationList();
    lc.updateDistanceMatrix();

    GAParameters gaP;
    gaP.populationSize = settings.getGaPopulation() * tc.getTaskToScheduleSize();
    gaP.maxIterations = settings.getGaIterations();
    gaP.noChangeLimit = settings.getGaNoChangeLimit();
    gaP.goalFitness = 1.0;
    gaP.elitismRate = settings.getGaElitism();
    gaP.mutationRate = settings.getGaMutation();

    auto tasks = tc.getTasksToSchedule();
    auto robots = rc.getFreeRobots();
    auto distance = lc.getDistanceMatrix();

    std::cout << "Running GA" << std::endl;
    Island is(gaP, settings.getGaSubIterations(), settings.getGaSubIterations(), tasks, robots, distance);
    is.solve();
    Chromosome best = is.getBest();
    best.printResult();

    std::map<uint32_t, system_server::MsgTaskList> listOfTaskList;
    std::vector<uint32_t> taskFailedId;
    best.getResult(listOfTaskList, taskFailedId);

    mtx->lock();
    //Update database and list of running tasks
    std::cout << "Updating GA results in database and running list" << std::endl;
    tc.updateTaskScheduled(listOfTaskList, taskFailedId);

    //Send results
    std::cout << "Sending GA results" << std::endl;
    for (auto r : listOfTaskList)
    {
        rc.sendTaskList(r.first, r.second);
    }
    ros::Duration d(2);
    d.sleep();
    ros::spinOnce();
    mtx->unlock();
}

void GeneralController::schedulingLoop()
{
    //not using duration because the thread sleeps too mutch
    ros::Rate r(5);
    auto start = std::chrono::system_clock::now();
    uint32_t attemps = 1;
    while (ros::ok())
    {
        auto end = std::chrono::system_clock::now();
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0;
        if (time >= (double)settings.getGaTimeInterval() / (double)attemps)
        {
            //It is in the begin, to its take into account processing below
            start = std::chrono::system_clock::now();

            tc.updateTasksToSchedule(settings.getTaskPoolSize());

            //if number of tasks is smaller than the poll, it decreases the time interval
            if (tc.getTaskToScheduleSize() < settings.getTaskPoolSize() && attemps < 3)
            {
                attemps++;
                ros::spinOnce();
            }
            //if is the third attemp or task list is bigger than 1, or equal to the task pool
            //try check robots
            //if it have task pool size ok, but robots pool size small, try three times too...
            //if it have tried three time and rave at least one robot and two tasks, runs scheduler
            else if (tc.getTaskToScheduleSize() > 1)
            {
                rc.searchFreeRobot(5);
                ros::spinOnce();
                if (rc.getFreeRobotListSize() < settings.getRobotPoolSize() && attemps < 3)
                {
                    attemps++;
                }
                else if (rc.getFreeRobotListSize() >= 1)
                {
                    callScheduler();
                    ros::spinOnce();
                    attemps = 1;
                }
            }
        }
        r.sleep();
        ros::spinOnce();
    }
}

void GeneralController::checkRobotLoop()
{
    //not using duration because the thread sleeps too mutch
    ros::Rate r(5);
    auto start = std::chrono::system_clock::now();
    while (ros::ok())
    {
        auto end = std::chrono::system_clock::now();
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0;
        if (time >= (double)settings.getRobotTimeInterval())
        {
            std::vector<uint32_t> failedRobots;
            rc.checkRobots(failedRobots);
            //In case of failed robots being listening, cancel all tasks
            for (auto r : failedRobots)
            {
                std::vector<uint32_t> failedTasksFromRobot;
                //Set as failed in task controller and database
                tc.setFail(r, failedTasksFromRobot, getCurrentTime_s());

                //send cancel requisition to the reboto
                for (auto t : failedTasksFromRobot)
                {
                    system_server::MsgRequest msg;
                    msg.type = system_server::MsgRequest::CANCEL_TASK;
                    msg.data = t;
                    rc.sendRequest(r, msg);
                }
            }
            start = std::chrono::system_clock::now();
        }
        r.sleep();
        ros::spinOnce();
    }
}

void GeneralController::checkTaskDeadlineLoop()
{
    //not using duration because the thread sleeps too mutch
    ros::Rate r(5);
    auto start = std::chrono::system_clock::now();
    while (ros::ok())
    {
        auto end = std::chrono::system_clock::now();
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0;
        if (time >= (double)settings.getTaskTimeInterval())
        {
            mtx->lock();
            std::vector<uint32_t> failedTasks;
            tc.deadlineCheck(failedTasks, getCurrentTime_s());
            mtx->unlock();
            for (auto t : failedTasks)
            {
                uint32_t idRobot;
                if (tc.getRobotincharge(t, idRobot))
                {
                    system_server::MsgRequest msg;
                    msg.type = system_server::MsgRequest::CANCEL_TASK;
                    msg.data = t;
                    rc.sendRequest(idRobot, msg);
                }
            }
            start = std::chrono::system_clock::now();
        }
        r.sleep();
        ros::spinOnce();
    }
}

void GeneralController::checkTaskToCancelLoop()
{
    //not using duration because the thread sleeps too mutch
    ros::Rate r(5);
    auto start = std::chrono::system_clock::now();
    while (ros::ok())
    {
        auto end = std::chrono::system_clock::now();
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0;
        if (time >= (double)settings.getTaskTimeInterval())
        {
            mtx->lock();
            std::vector<uint32_t> tasksToCancel;
            tc.toCancelCheck(tasksToCancel);
            mtx->unlock();
            for (auto t : tasksToCancel)
            {
                uint32_t idRobot;
                if (tc.getRobotincharge(t, idRobot))
                {
                    system_server::MsgRequest msg;
                    msg.type = system_server::MsgRequest::CANCEL_TASK;
                    msg.data = t;
                    rc.sendRequest(idRobot, msg);
                }
            }
            start = std::chrono::system_clock::now();
        }
        r.sleep();
        ros::spinOnce();
    }
}

void GeneralController::callbackRequestRobotCheck(const system_server::MsgRequest &msg)
{
    rc.checkRobot(msg.data);
}

void GeneralController::callbackRequestChargeBattery(const system_server::MsgRequest &msg)
{
    Robot *rbt;
    rbt = rc.getRobotById(msg.data);
    if (rbt != NULL)
    {
        system_server::MsgRequest msgResponse;
        msgResponse.type = system_server::MsgRequest::CHARGE_BATTERY;
        msgResponse.data = rbt->getDepot();
        rc.sendRequest(msg.data, msgResponse);
    }
}

void GeneralController::callbackRequestCancelTask(const system_server::MsgRequest &msg)
{
    //update robot in charge
    uint32_t idRobot;
    if (tc.getRobotincharge(msg.data, idRobot))
        rc.checkRobot(msg.data);

    tc.updateTaskStatus(msg.data, Task::STATUS_CANCELLED, getCurrentTime_s());
}

void GeneralController::callbackRequestPerformingPickUp(const system_server::MsgRequest &msg)
{
    //update robot in charge
    uint32_t idRobot;
    if (tc.getRobotincharge(msg.data, idRobot))
        rc.checkRobot(msg.data);

    tc.updateTaskStatus(msg.data, Task::STATUS_PERFORMING_PICK_UP, getCurrentTime_s());
}

void GeneralController::callbackRequestPerformingDelivery(const system_server::MsgRequest &msg)
{
    //update robot in charge
    uint32_t idRobot;
    if (tc.getRobotincharge(msg.data, idRobot))
        rc.checkRobot(msg.data);

    tc.updateTaskStatus(msg.data, Task::STATUS_PERFORMING_DELIVERY);
}

void GeneralController::callbackRequestSucessTask(const system_server::MsgRequest &msg)
{
    //update robot in charge
    uint32_t idRobot;
    if (tc.getRobotincharge(msg.data, idRobot))
        rc.checkRobot(msg.data);

    tc.updateTaskStatus(msg.data, Task::STATUS_SUCESS, getCurrentTime_s());
}

void GeneralController::callbackRequestFailTask(const system_server::MsgRequest &msg)
{
    //update robot in charge
    uint32_t idRobot;
    if (tc.getRobotincharge(msg.data, idRobot))
        rc.checkRobot(msg.data);

    tc.updateTaskStatus(msg.data, Task::STATUS_FAILED, getCurrentTime_s());
}

void GeneralController::callbackRequest(const system_server::MsgRequest &msg)
{
    switch (msg.type)
    {
    case system_server::MsgRequest::ROBOT_CHECK:
        callbackRequestRobotCheck(msg);
        break;

    case system_server::MsgRequest::CHARGE_BATTERY:
        callbackRequestChargeBattery(msg);
        break;

    case system_server::MsgRequest::CANCEL_TASK:
        callbackRequestCancelTask(msg);
        break;

    case system_server::MsgRequest::PERFORMING_PICK_UP:
        callbackRequestPerformingPickUp(msg);
        break;

    case system_server::MsgRequest::PERFORMING_DELIVERY:
        callbackRequestPerformingDelivery(msg);
        break;

    case system_server::MsgRequest::SUCESS_TASK:
        callbackRequestSucessTask(msg);
        break;

    case system_server::MsgRequest::FAIL_TASK:
        callbackRequestFailTask(msg);
        break;

        //case system_server::MsgRequest::FREE_ROBOT:
        //do nothing
        //break;

    default:
        break;
    }
}

void GeneralController::callbackRobotData(const system_server::MsgRobotData &msg)
{
    rc.checkRobot(msg.id);
    rc.callbackRobotData(msg);
}

void GeneralController::run()
{
    //Dont remove
    sDao.getSettings(0, settings);
    lc.updateLocationList();
    lc.updateDistanceMatrix();
    rc.updateAllRobots();
    rc.updateFreeRobots();
    //lc.run();

    subRequest = nh.subscribe("server_request", 100, &GeneralController::callbackRequest, this);
    subRobotData = nh.subscribe("server_robot_data", 100, &GeneralController::callbackRobotData, this);

    std::thread scLoopThr(&GeneralController::schedulingLoop, this);
    std::thread chkRbtLoopThr(&GeneralController::checkRobotLoop, this);
    std::thread chkTaskDeadlineLoopThr(&GeneralController::checkTaskDeadlineLoop, this);
    std::thread chkTaskToCancelLoopThr(&GeneralController::checkTaskToCancelLoop, this);
    ros::spin();

    return;

    lc.run();
    ros::spinOnce();
    lc.updateLocationList();
    lc.updateDistanceMatrix();
    rc.updateAllRobots();

    ros::Duration d(5);
    //d.sleep();

    auto c = tc.generateTasksCoordination(rc.getAllRobots(), lc.getDistanceMatrix());

    auto tl = tc.getTasksToSchedule();
    for (auto t : *tl)
    {
        std::cout << t;
    }
    c.printResult();

    double dis = lc.getDistance(2, 100);
    dis += lc.getDistance(100, 28);
    dis += lc.getDistance(28, 195);
    dis += lc.getDistance(195, 125);

    std::cout << "Batt = " << (dis / 0.5) * (1.0 + 10.0 / 100.0) << std::endl;

    ros::spin();
}