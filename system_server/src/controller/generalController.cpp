#include "generalController.hpp"

GeneralController::GeneralController(std::string host, std::string user, std::string pass, std::string database)
{
    this->gdao = GeneralDao(host, user, pass, database);
    this->rc = RobotController(&gdao);
    this->tc = TaskController(&gdao);
}

GeneralController::~GeneralController() {}

void GeneralController::callScheduler()
{
    auto& rVec = rc.getFreeRobots();
    auto& tVec = tc.getTasksToSchedule();

    std::cout << "Robots: ";
    for (auto r : rVec)
    {
        std::cout << r.getId() << " ";
    }

    std::cout << "\nTaks: ";
    for (auto t : tVec)
    {
        std::cout << t.getId() << " ";
    }    
}