#ifndef TEST_CONTROLLER_HPP
#define TEST_CONTROLLER_HPP

#include "locationController.hpp"
#include "robotController.hpp"
#include "taskController.hpp"

class TestController
{
private:
    /* data */
public:
    TestController(/* args */);
    ~TestController();
    void static hitRateTest(RobotController* rc, TaskController* tc, LocationController* lc);
};

TestController::TestController(/* args */)
{
}

TestController::~TestController()
{
}


#endif