#ifndef ROBOT_DAO_HPP
#define ROBOT_DAO_HPP

#include "generalDao.hpp"
#include "../model/robot.hpp"

typedef struct _robotRequestData
{
    uint32_t id;
    std::string status;
    uint32_t currentLocation;
    double remainingBattery;
    double mediumVelocity;
} RobotRequestData;

class RobotDao
{
private:
    GeneralDao *gDao;

public:
    RobotDao();
    RobotDao(GeneralDao *gDao);
    ~RobotDao();
    void setGeneralDao(GeneralDao *gDao);
    bool getRobotList(std::vector<Robot> &robotList, bool equalToStatus, std::string status);
    bool updateRobot(Robot &robot);
    bool updateRobot(std::vector<Robot> &robotList);
    bool updateRobotRequest(RobotRequestData robotRequestData);
};

#endif