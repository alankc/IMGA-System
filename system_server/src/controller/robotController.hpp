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
    RobotController();
    RobotController(GeneralDao *gDao);
    ~RobotController();

    void updateAllRobots();
    void updateFreeRobots();

    Robot *getRobotById(uint32_t id);
    Robot *getRobotByIndex(uint32_t index);
    void copyRobotList(std::vector<Robot> &copy);
    std::size_t getRobotListSize();    
};

#endif