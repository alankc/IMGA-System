#include "robotController.hpp"

#include <random>

RobotController::RobotController() {}

RobotController::RobotController(GeneralDao *gDao)
{
    this->rd = RobotDao(gDao);
}

RobotController::~RobotController() {}

void RobotController::updateAllRobots()
{
    allRobots.clear();
    bool tst = rd.getRobotList(allRobots, false, Robot::STATUS_FAIL);
    if (!tst)
        std::cout << "Fail to update all robots list" << std::endl;
    else
    {
        pubRequest.clear();
        pubTaskList.clear();
        for (auto r : allRobots)
        {
            std::string topic = "/robot" + std::to_string(r.getId());
            pubRequest[r.getId()] = nh.advertise<system_server::MsgRequest>(topic + "/requests", 100);
            pubTaskList[r.getId()] = nh.advertise<system_server::MsgTaskList>(topic + "/task_list", 100);
        }
        std::cout << "All robots list has been updated" << std::endl;
    }
}

void RobotController::updateFreeRobots()
{
    freeRobots.clear();
    for (auto r : allRobots)
    {
        if (r.getStatus() == Robot::STATUS_FREE)
            freeRobots.push_back(r);
    }
}

void RobotController::searchFreeRobot(double waitingTime_s)
{
    for (auto pub : pubRequest)
    {
        system_server::MsgRequest msg;
        msg.type = system_server::MsgRequest::FREE_ROBOT;
        msg.data = 0;
        pub.second.publish(msg);
        ros::spinOnce();
    }
    ros::Duration d(waitingTime_s);
    d.sleep();
    ros::spinOnce();
}

void RobotController::sendRequest(uint32_t idRobot, system_server::MsgRequest &msg)
{
    auto pub = pubRequest.find(idRobot);
    if (pub != pubRequest.end())
    {
        pub->second.publish(msg);
        ros::spinOnce();
    }
}
void RobotController::sendTaskList(uint32_t idRobot, system_server::MsgTaskList &msg)
{
    auto pub = pubTaskList.find(idRobot);
    if (pub != pubTaskList.end())
    {
        pub->second.publish(msg);
        ros::spinOnce();
    }
}

void RobotController::callbackRobotData(const system_server::MsgRobotData &msg)
{
    auto allRobotsIt = std::find(allRobots.begin(), allRobots.end(), msg.id);

    //Must exist in the list of robots
    if (allRobotsIt != allRobots.end())
    {
        allRobotsIt->setRemainingBattery(msg.battery);
        allRobotsIt->setCurrentLocation(msg.currLocation);
        allRobotsIt->setMediumVelocity(msg.minSpeed);
        allRobotsIt->setStatus(msg.status);

        //update database
        rd.updateRobotRequest(const_cast<system_server::MsgRobotData &>(msg));
    }
}

std::vector<Robot> *RobotController::getFreeRobots()
{
    return &freeRobots;
}

std::vector<Robot> *RobotController::getAllRobots()
{
    return &allRobots;
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

Robot *RobotController::getRobotById(uint32_t id)
{
    auto it = std::find(allRobots.begin(), allRobots.end(), id);

    if (it != allRobots.end())
        return &(*it);

    return NULL;
}

Robot *RobotController::getRobotByIndex(uint32_t index)
{
    return &allRobots[index];
}

void RobotController::copyFreeRobotList(std::vector<Robot> &copy)
{
    copy = std::vector<Robot>(freeRobots);
}

std::size_t RobotController::getFreeRobotListSize()
{
    return freeRobots.size();
}

void RobotController::generateRobots(uint32_t numberOfRobots, uint32_t minpayload, uint32_t maxPayload, double minSpeed, double maxSpeed, double minDisFac, double maxDisFac, uint32_t numberOfDepots)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint32_t> rndPayload(minpayload, maxPayload);
    std::uniform_int_distribution<uint32_t> rndDepot(0, numberOfDepots - 1);
    std::uniform_real_distribution<double> rndSpeed(minSpeed, maxSpeed);
    std::uniform_real_distribution<double> rndDisFac(minDisFac, maxDisFac);

    freeRobots.clear();
    for (uint32_t i = 0; i < numberOfRobots; i++)
    {
        Robot r(i);
        r.setDescription("Robot " + std::to_string(i));
        r.setStatus(Robot::STATUS_FREE);
        r.setMaximumPayload(rndPayload(gen));
        uint32_t depot = rndDepot(gen);
        r.setCurrentLocation(depot);
        r.setDepot(depot);
        r.setMediumVelocity(rndSpeed(gen));
        r.setDischargeFactor(rndDisFac(gen)); //por fazer
        r.setBatteryThreshold(0);
        r.setRemainingBattery(0);
        freeRobots.push_back(r);
    }
}
