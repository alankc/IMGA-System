#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <vector>

#include "../model/robot.hpp"
#include "../dao/robotDao.hpp"

class RobotController
{
private:
    RobotDao rd;
    uint32_t id;
    Robot r;
public:
    RobotController();
    RobotController(GeneralDao *gDao, uint32_t id);
    ~RobotController();

    bool updateRobotFromDB();  
    Robot* getRobot();
};

#endif