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
}

GeneralController::~GeneralController() {}

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
    gaP.populationSize = settings.getGaPopulation();
    gaP.maxIterations = settings.getGaIterations();
    gaP.noChangeLimit = settings.getGaNoChangeLimit();
    gaP.goalFitness = 1.0;
    gaP.elitismRate = settings.getGaElitism();
    gaP.mutationRate = settings.getGaMutation();

    auto tasks = tc.getTaskList();
    auto robots = rc.getFreeRobots();
    auto distance = lc.getDistanceMatrix();

    Island is(gaP, settings.getGaSubIterations(), settings.getGaSubIterations(), tasks, robots, distance);
    is.solve();
    Chromosome best = is.getBest();
    best.printResult();

    //Send results
    std::map<uint32_t, system_server::MsgTaskList> listOfTaskList; 
    std::vector<uint32_t> taskFailedId;
    best.getResult(listOfTaskList, taskFailedId);
    for (auto r : listOfTaskList)
    {
        rc.sendTaskList(r.first, r.second);
    }

    //Update database
    std::cout << "FALTA ATUALIZAR AS TAREFAS NO BANCO" << std::endl;
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
        if (time >= (double)settings.getTimeInterval() / (double)attemps)
        {
            //It is in the begin, to its take into account processing below
            start = std::chrono::system_clock::now();

            tc.updateTasksToSchedule(settings.getTaskPoolSize());

            //if number of tasks is smaller than the poll, it decreases the time interval
            if (tc.getTaskListSize() < settings.getTaskPoolSize() && attemps < 3)
            {
                attemps++;
            }
            //if is the third attemp or task list is bigger than 1, or equal to the task pool
            //try check robots
            //if it have task pool size ok, but robots pool size small, try three times too...
            //if it have tried three time and rave at least one robot and two tasks, runs scheduler
            else if (tc.getTaskListSize() > 1)
            {
                rc.searchFreeRobot(5);
                if (rc.getFreeRobotListSize() < settings.getRobotPoolSize() && attemps < 3)
                {
                    attemps++;
                }
                else if (rc.getFreeRobotListSize() >= 1)
                {
                    callScheduler();
                    attemps = 1;
                }
            }
        }
        r.sleep();
    }
}

void GeneralController::run()
{
    //Dont remove
    sDao.getSettings(0, settings);
    //lc.updateLocationList();
    //lc.updateDistanceMatrix();
    rc.updateAllRobots();
    //lc.run();

    ros::NodeHandle nh;
    auto pub = nh.subscribe("/robot_data", 100, &RobotController::callbackRobotData, &rc);

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

    auto tl = tc.getTaskList();
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