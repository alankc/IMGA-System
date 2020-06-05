#ifndef GENERAL_CONTROLLER_HPP
#define GENERAL_CONTROLLER_HPP

#include "robotController.hpp"
#include "taskController.hpp"
#include "locationController.hpp"

class GeneralController
{
private:
    GeneralDao gdao;
    RobotController rc;
    TaskController tc;
    LocationController lc;

public:
    GeneralController(std::string host, std::string user, std::string pass, std::string database);
    ~GeneralController();

    void callScheduler();
    void callUpdateRobots();
    void listnerupdatesRobots();
};

#endif