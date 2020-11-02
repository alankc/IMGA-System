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

    //Update database
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
                rc.updateFreeRobots();
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
    sDao.getSettings(0, settings);

    TaskDao taskDaoTst(&gdao);

    taskDaoTst.updateTask(0, TaskDao::Column::description, "Teste");
    taskDaoTst.updateTask(0, TaskDao::Column::status, "TT");

    for (uint32_t i = TaskDao::Column::pickUpLocation; i <= TaskDao::Column::endTime; i++)
    {
        if (static_cast<TaskDao::Column>(i) != TaskDao::Column::robotInCharge)
            taskDaoTst.updateTask(0, static_cast<TaskDao::Column>(i), std::to_string(i));
        else
            taskDaoTst.updateTask(0, static_cast<TaskDao::Column>(i), "2");
    }

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

/*
All robots list has been updated
0,Task 0,N,2,44.6602,161,131,0
1,Task 1,N,2,112.098,151,180,0
2,Task 2,N,2,240.316,143,32,0
3,Task 3,N,2,367.468,187,27,0
4,Task 4,N,2,122.8,110,188,1
5,Task 5,N,2,216.09,140,175,1
6,Task 6,N,2,321.664,203,105,1
7,Task 7,N,5,119.109,79,104,2
8,Task 8,N,5,193.306,51,80,2
9,Task 9,N,5,281.037,56,37,2
10,Task 10,N,5,147.489,199,45,3
11,Task 11,N,5,262.992,138,143,3
12,Task 12,N,5,389.214,180,43,3
13,Task 13,N,10,98.4929,100,28,4
14,Task 14,N,10,192.252,195,125,4
    Robots:   0  0  0  0  1  1  1  2  2  2  3  3  3  4  4
     Tasks:   0  1  2  3  4  5  6  7  8  9 10 11 12 13 14
    Failed:  14
   Fitness: 290.124


*/