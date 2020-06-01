#include "robotController.hpp"

RobotController::RobotController(GeneralDao *gDao) {}

RobotController::RobotController(GeneralDao *gDao)
{
    this->rd = RobotDao(gDao);
}

RobotController::~RobotController() {}

std::vector<Robot> &RobotController::getAllRobots()
{
    return allRobots;
}

std::vector<Robot> &RobotController::getFreeRobots()
{
    return freeRobots;
}

void RobotController::updateAllRobots()
{
}

void RobotController::updateFreeRobots()
{
    freeRobots.clear();
    bool tst = rd.getRobotList(freeRobots, Robot::STATUS_FREE);
    if (!tst)
        std::cout << "Fail to update free robots list" << std::endl;
    else
        std::cout << "Free robots list has been updated" << std::endl;
}