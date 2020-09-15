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

    std::vector<Robot> *getFreeRobots();
    Robot *getFreeRobotById(uint32_t id);
    Robot *getFreeRobotByIndex(uint32_t index);
    void copyFreeRobotList(std::vector<Robot> &copy);
    std::size_t getFreeRobotListSize();    
    void generateRobots(uint32_t numberOfRobots, uint32_t minpayload, uint32_t maxPayload, double minSpeed, double maxSpeed, double minDisFac, double maxDisFac, uint32_t numberOfDepots);
};

#endif