#include "robotController.hpp"

RobotController::RobotController() {}

RobotController::RobotController(GeneralDao *gDao)
{
    this->rd = RobotDao(gDao);
}

RobotController::~RobotController() {}

void RobotController::updateAllRobots()
{
    allRobots.clear();
    bool tst = rd.getRobotList(allRobots, "");
    if (!tst)
        std::cout << "Fail to update all robots list" << std::endl;
    else
        std::cout << "All robots list has been updated" << std::endl;
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

Robot *RobotController::getFreeRobotById(uint32_t id)
{
    auto it = std::find(freeRobots.begin(), freeRobots.end(), id);

    if (it != freeRobots.end())
        return &(*it);

    return NULL;
}

Robot *RobotController::getFreeRobotByIndex(uint32_t index)
{
    return &freeRobots[index];
}

void RobotController::copyFreeRobotList(std::vector<Robot> &copy)
{
    copy = std::vector<Robot>(freeRobots);
}

std::size_t RobotController::getFreeRobotListSize()
{
    return freeRobots.size();
}