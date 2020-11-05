#ifndef ROBOT_DAO_HPP
#define ROBOT_DAO_HPP

#include "generalDao.hpp"
#include "../model/robot.hpp"
#include <system_server/MsgRobotData.h>

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
    bool updateRobotRequest(system_server::MsgRobotData &robotRequestData);

    enum Column
    {
        id = 0,
        description,
        status,
        depot,
        currentLocation,
        maximumPayload,
        remainingBattery,
        dischargeFactor,
        batteryThreshold,
        mediumVelocity,
        waitingTime
    };

    bool updateRobot(uint32_t id, RobotDao::Column column, std::string data);
};

#endif