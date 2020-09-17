#include "testController.hpp"
#include "../scheduler/island.hpp"
#include <chrono>

RobotController *TestController::rc = NULL;
TaskController *TestController::tc = NULL;
LocationController *TestController::lc = NULL;

TestController::TestController(/* args */) {}

TestController::~TestController() {}

void TestController::setControllers(RobotController *rc, TaskController *tc, LocationController *lc)
{
    TestController::rc = rc;
    TestController::tc = tc;
    TestController::lc = lc;
}

void TestController::hitRateTest(uint32_t repetitions, double min, double max, double step)
{
    std::cout << "Elitism"
              << "\t"
              << "Mutation"
              << "\t"
              << "AvgTime"
              << "\t"
              << "stdTime"
              << "\t"
              << "HitRate"
              << "\t"
              << "IslandsHitRate" << std::endl;

    for (double elitism = min; elitism <= max + step * 0.1; elitism += step)
    {
        for (double mutation = min; mutation <= max + step * 0.1; mutation += step)
        {
            elitism = std::round(elitism * 100) / 100;
            mutation = std::round(mutation * 100) / 100;

            std::cout << elitism << "\t" << mutation << "\t";

            GAParameters gaP;
            gaP.populationSize = 100;
            gaP.mutationRate = mutation;
            gaP.elitismRate = elitism;
            gaP.maxIterations = 20;
            gaP.noChangeLimit = gaP.maxIterations;

            std::vector<double> islandHitRate(36, 0.0);
            double globalHitRate = 0.0;
            std::vector<int64_t> timeDuration(repetitions);

            for (uint32_t i = 0; i < repetitions; i++)
            {
                lc->gerenateLocations(10, 3, 10, 50);
                auto d = lc->getDistanceMatrix();
                rc->generateRobots(5, 1, 10, 0.5, 3, 1.0 / (8 * 60 * 60), 1.0 / (4 * 60 * 60), 3);
                auto r = rc->getFreeRobots();
                auto crResult = tc->generateTasks(15, r, d, 3);

                auto tasks = tc->getTaskList();
                auto robots = rc->getFreeRobots();
                auto distance = lc->getDistanceMatrix();

                Island is(gaP, 50, 0.1, tasks, robots, distance);

                auto start = std::chrono::high_resolution_clock::now();
                is.solve();
                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

                timeDuration[i] = duration.count();

                Chromosome best = is.getBest();
                if (best.allScheduled())
                    globalHitRate += 1.0 / (double)repetitions;

                auto bestList = is.getListOfBests();
                for (uint32_t j = 0; j < bestList.size(); j++)
                    if (bestList[j].allScheduled())
                        islandHitRate[j] += 1.0 / (double)repetitions;
            }

            double avgTime = computeMean<int64_t>(timeDuration);
            double stdTime = std::sqrt(computeVariance<int64_t>(timeDuration, avgTime));

            std::cout << avgTime << "\t" << stdTime << "\t" << globalHitRate;

            for (uint32_t i = 0; i < islandHitRate.size(); i++)
                std::cout << "\t" << islandHitRate[i];

            std::cout << std::endl;
        }
    }
}