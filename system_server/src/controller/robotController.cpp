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

std::vector<Robot> *RobotController::getFreeRobots()
{
    return &freeRobots;
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
        freeRobots.push_back(r);
    }
}