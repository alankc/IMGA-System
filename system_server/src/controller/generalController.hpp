#ifndef GENERAL_CONTROLLER_HPP
#define GENERAL_CONTROLLER_HPP

#include "robotController.hpp"
#include "taskController.hpp"

class GeneralController
{
private:
    GeneralDao gdao;
    RobotController rc;
    TaskController tc;

public:
    GeneralController(std::string host, std::string user, std::string pass, std::string database);
    ~GeneralController();

    void callScheduler();
};

#endif