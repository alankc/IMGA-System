#include "generalController.hpp"
#include "../scheduler/island.hpp"

GeneralController::GeneralController()
{
}

GeneralController::GeneralController(std::string host, std::string user, std::string pass, std::string database)
{
    this->gdao = GeneralDao(host, user, pass, database);
    this->rc = RobotController(&gdao);
    this->tc = TaskController(&gdao);
    this->lc = LocationController(&gdao);
}

GeneralController::~GeneralController() {}

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
    TestController tstController;
    tstController.setControllers(&rc, &tc, &lc);

    //TestController::hitRateTest(1000, 0.05, 0.3, 0.050);
    //TestController::hitRateTest(100, 0.05, 0.05, 0.050);
    //TestController::Experiment2(2, 0.15, 0.15, 0.05, 60, 100, 10);
    //TestController::Experiment2(1000, 0.20, 0.30, 0.05, 10, 100, 10);
    TestController::Experiment3(10, 5, 30, 5);
    TestController::Experiment3(10, 40, 50, 10);
}