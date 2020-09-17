#ifndef TEST_CONTROLLER_HPP
#define TEST_CONTROLLER_HPP

#include "locationController.hpp"
#include "robotController.hpp"
#include "taskController.hpp"
#include <cmath>

class TestController
{
private:
    static RobotController *rc;
    static TaskController *tc;
    static LocationController *lc;

    template <typename T>
    static double computeMean(std::vector<T> &data);
    template <typename T>
    static double computeVariance(std::vector<T> &data, double mean);

public:
    TestController();
    ~TestController();

    void static setControllers(RobotController *rc, TaskController *tc, LocationController *lc);
    /*
    * Evaluate Hit Rate of islands varing mutation and elitsm
    *
    */
    void static hitRateTest(uint32_t repetitions, double min, double max, double step);
};

template <typename T>
double TestController::computeMean(std::vector<T> &data)
{
    double mean = 0;

    for (auto d : data)
        mean += d;
    mean = mean / data.size();

    return mean;
}

template <typename T>
double TestController::computeVariance(std::vector<T> &data, double mean)
{
    double variance = 0;

    for (auto d : data)
        variance += std::pow(d - mean, 2);
    variance = variance / (data.size() - 1.0);

    return variance;
}

#endif