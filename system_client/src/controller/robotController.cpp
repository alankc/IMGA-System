#include "robotController.hpp"

RobotController::RobotController() {}

RobotController::RobotController(GeneralDao *gDao, uint32_t id)
{
    this->rd = RobotDao(gDao);
    this->id = id;
}

RobotController::~RobotController() {}

bool RobotController::updateRobotFromDB()
{
    return rd.getRobot(id, r);
}

Robot* RobotController::getRobot()
{
    return &r;
}