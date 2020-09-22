#ifndef TEST_CONTROLLER_HPP
#define TEST_CONTROLLER_HPP

#include "locationController.hpp"
#include "robotController.hpp"
#include "taskController.hpp"
#include <cmath>

typedef struct _Exp3Result
{
	int64_t timeToScheduling = 0;
	double scheduledPercentage = 0.0;

	double energyExpected = 0.0;
	double energyObtained = 0.0;
	double timeExpected = 0.0;
	double timeObtained = 0.0;
 	uint64_t payloadExpected = 0;
	uint64_t payloadObtained = 0;

	double meanNormEnergyE = 0.0;
	double stdNormEnergyE = 0.0;
	double meanNormEnergyO = 0.0;
	double stdNormEnergyO = 0.0;
	
	double meanNormPayloadE = 0.0;
	double stdNormPayloadE = 0.0;	
	double meanNormPayloadO = 0.0;	
	double stdNormPayloadO = 0.0;	
} Exp3Result;

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
    * Evaluate Hit Rate of islands varing mutation and elitsm from islands
    */
    void static hitRateTest(uint32_t repetitions, double min, double max, double step);

    /*
    * Evaluate Hit Rate of islands varing migration and subiterations 
    */
    void static Experiment2(uint32_t repetitions, double minMig, double maxMig, double stepMig, uint32_t minSubIt, uint32_t maxSubIt, uint32_t stepSubIt);
   
    /*
    * Evaluate Hit Rate of islands varing tasnks and robots
    *   Number of robots is defined by a function
    */
    void static Experiment3(uint32_t repetitions, uint32_t minT, uint32_t maxT, uint32_t stepT);
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