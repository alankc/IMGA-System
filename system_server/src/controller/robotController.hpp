#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <ros/ros.h>
#include <vector>
#include <map>
#include <system_server/MsgRobotData.h>
#include <system_server/MsgRequest.h>
#include <system_server/MsgTaskList.h>

#include "../model/robot.hpp"
#include "../dao/robotDao.hpp"

class RobotController
{
private:
    RobotDao rd;
    std::vector<Robot> allRobots;
    std::vector<Robot> freeRobots;
    //std::map<id from robot, number of checks>
    std::map<uint32_t, uint8_t> robotsCheckCounter;

    ros::NodeHandle nh;
    //std::map<id from robot,ros::Publisher to the robot>
    std::map<uint32_t, ros::Publisher> pubRequest;
    std::map<uint32_t, ros::Publisher> pubTaskList;

public:
    RobotController();
    RobotController(GeneralDao *gDao);
    ~RobotController();

    void updateAllRobots();
    void updateFreeRobots();
    void searchFreeRobot(double waitingTime_s);
    void checkRobots(std::vector<uint32_t> &idRobotFailed);
    void checkRobot(uint32_t idRobot);
    void sendRequest(uint32_t idRobot, system_server::MsgRequest &msg);
    void sendTaskList(uint32_t idRobot, system_server::MsgTaskList &msg);
    void callbackRobotData(const system_server::MsgRobotData &msg);
    bool updateRobot(uint32_t id, RobotDao::Column column, std::string data);

    std::vector<Robot> *getFreeRobots();
    std::vector<Robot> *getAllRobots();
    Robot *getFreeRobotById(uint32_t id);
    Robot *getFreeRobotByIndex(uint32_t index);
    Robot *getRobotById(uint32_t id);
    Robot *getRobotByIndex(uint32_t index);
    void copyFreeRobotList(std::vector<Robot> &copy);
    std::size_t getFreeRobotListSize();
    void generateRobots(uint32_t numberOfRobots, uint32_t minpayload, uint32_t maxPayload, double minSpeed, double maxSpeed, double minDisFac, double maxDisFac, uint32_t numberOfDepots);
};

#endif