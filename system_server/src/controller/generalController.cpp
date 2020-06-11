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
    rc.updateFreeRobots();
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
    best.printResult();
}