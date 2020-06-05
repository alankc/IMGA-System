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
    rc.updateFreeRobots();
    tc.updateTasksToSchedule();
    
    std::vector<Robot> robots;
    std::vector<Task> tasks;

    rc.copyFreeRobotList(robots);
    tc.copyTaskList(tasks);

    std::cout << "Robots: ";
    for (auto r : robots)
    {
        std::cout << r.getId() << " ";
    }

    std::cout << "\nTaks: ";
    for (auto t : tasks)
    {
        std::cout << t.getId() << " ";
    }    
}