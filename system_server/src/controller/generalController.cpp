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

    lc.gerenateLocations(6, 3, 10, 50);
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
}