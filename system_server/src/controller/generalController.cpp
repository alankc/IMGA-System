#include "generalController.hpp"

GeneralController::GeneralController()
{}

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
      
}