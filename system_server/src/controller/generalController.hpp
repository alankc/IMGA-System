#ifndef GENERAL_CONTROLLER_HPP
#define GENERAL_CONTROLLER_HPP

#include "robotController.hpp"
#include "taskController.hpp"
#include "../model/location.hpp"

class GeneralController
{
private:
    GeneralDao gdao;
    RobotController rc;
    TaskController tc;

    std::vector<Location> locationList;
    std::vector<std::vector<double>> distanceMatrix;

public:
    GeneralController(std::string host, std::string user, std::string pass, std::string database);
    ~GeneralController();

    void callScheduler();
};

#endif