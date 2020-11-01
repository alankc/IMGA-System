#include "generalController.hpp"
#include "../scheduler/island.hpp"

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
    /*rc.updateFreeRobots();
    tc.updateTasksToSchedule();
    lc.updateLocationList();
    lc.updateDistanceMatrix();

    GAParameters gaP;
    gaP.populationSize = 500;
    gaP.mutationRate = 0.25;
    gaP.elitismRate = 0.25;
    gaP.maxIterations = 3000;
    gaP.noChangeLimit = 0.5 * gaP.maxIterations;

    auto tasks = tc.getTaskList();
    auto robots = rc.getFreeRobots();
    auto distance = lc.getDistanceMatrix();

    Island is(gaP, 100, 0.1, tasks, robots, distance);
    is.solve();
    Chromosome best = is.getBest();
    best.printResult();*/

    /*lc.gerenateLocations(10, 3, 10, 50);
    for (uint32_t i = 0; i < lc.getLocationListSize(); i++)
    {
        std::cout << lc.getLocationByIndex(i)->getId() << " "
                  << lc.getLocationByIndex(i)->getDescription() << " depot = "
                  << lc.getLocationByIndex(i)->getIsDepot() << std::endl;
    }

    auto d = lc.getDistanceMatrix();

    for (uint32_t i = 0; i < lc.getLocationListSize(); i++)
    {
        for (uint32_t j = 0; j < lc.getLocationListSize(); j++)
        {
            std::cout << d->at(i).at(j) << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "Robots" << std::endl;
    rc.generateRobots(5, 1, 10, 0.5, 3, 1.0 / (8 * 60 * 60), 1.0 / (4 * 60 * 60), 3);
    auto r = rc.getFreeRobots();
    for (auto it : *r)
    {
        std::cout << it.getId() << " "
                  << it.getDescription() << " "
                  << it.getMaximumPayload() << " "
                  << it.getCurrentLocation() << " "
                  << it.getDepot() << " "
                  << it.getMediumVelocity() << " "
                  << it.getDischargeFactor() << std::endl;
    }

    std::cout << "Tasks" << std::endl;
    auto crResult = tc.generateTasks(20, r, d, 3);
    auto t = tc.getTaskList();
    for (auto it : *t)
    {
        std::cout << it;
    }

    std::cout << "Chromossome" << std::endl;

    crResult.printResult();*/
    double hitRate = 0.0;

    for (uint32_t i = 0; i < 1; i++)
    {
        lc.gerenateLocations(10, 3, 10, 50);
        auto d = lc.getDistanceMatrix();
        rc.generateRobots(5, 1, 10, 0.5, 3, 1.0 / (8 * 60 * 60), 1.0 / (4 * 60 * 60), 3);
        auto r = rc.getFreeRobots();
        //auto crResult = tc.generateTasks(20, r, d, 3);

        GAParameters gaP;
        gaP.populationSize = 100;
        gaP.mutationRate = 0.25;
        gaP.elitismRate = 0.05;
        gaP.maxIterations = 10;
        gaP.noChangeLimit = gaP.maxIterations;

        auto tasks = tc.getTaskList();
        auto robots = rc.getFreeRobots();
        auto distance = lc.getDistanceMatrix();

        Island is(gaP, 50, 0.1, tasks, robots, distance);
        is.solve();
        Chromosome best = is.getBest();
        best.printResult();

        auto bestList = is.getListOfBests();
        for (uint32_t j = 0; j < bestList.size(); j++)
            std::cout << bestList[j].allScheduled() << " ";

        std::cout << std::endl;
    }

    std::cout << "HitHate = " << hitRate << std::endl;
}

#include "testController.hpp"
void GeneralController::run()
{
    sDao.getSettings(0, settings);

    std::cout << settings.getIdSetup() << " ";
    std::cout << settings.getTaskPoolSize() << " ";
    std::cout << settings.getRobotPoolSize() << " ";
    std::cout << settings.getTimeInterval() << " ";
    std::cout << settings.getGaIterations() << " ";
    std::cout << settings.getGaSubIterations() << " ";
    std::cout << settings.getGaPopulation() << " ";
    std::cout << settings.getGaTimeLimit() << " ";
    std::cout << settings.getGaNoChangeLimit() << " ";
    std::cout << settings.getGaElitism() << " ";
    std::cout << settings.getGaMutation() << " ";
    std::cout << settings.getGaMigration() << std::endl;

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