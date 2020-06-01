#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <vector>

#include "../model/robot.hpp"
#include "../dao/robotDao.hpp"

class RobotController
{
private:
    RobotDao rd;
    std::vector<Robot> allRobots;
    std::vector<Robot> freeRobots;

public:
    RobotController(GeneralDao *gDao);
    ~RobotController();

    std::vector<Robot>& getAllRobots();
    std::vector<Robot>& getFreeRobots();

    void updateAllRobots();
    void updateFreeRobots();
};

#endif