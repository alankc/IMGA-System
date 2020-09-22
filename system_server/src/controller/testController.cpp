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

void TestController::Experiment2(uint32_t repetitions, double minMig, double maxMig, double stepMig, uint32_t minSubIt, uint32_t maxSubIt, uint32_t stepSubIt)
{
    std::cout << "Migration"
              << "\t"
              << "Subiteration"
              << "\t"
              << "AvgTime"
              << "\t"
              << "stdTime"
              << "\t"
              << "HitRate"
              << "\t"
              << "IslandsHitRate" << std::endl;

    for (double migration = minMig; migration <= maxMig + stepMig * 0.1; migration += stepMig)
    {
        for (uint32_t subIt = minSubIt; subIt <= maxSubIt; subIt += stepSubIt)
        {
            migration = std::round(migration * 100) / 100;

            std::cout << migration << "\t" << subIt << "\t";

            GAParameters gaP;
            gaP.populationSize = 100;
            gaP.mutationRate = 0.25;
            gaP.elitismRate = 0.05;
            gaP.maxIterations = std::round(1000.0 / subIt); //1000 global iteration total
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

                Island is(gaP, subIt, migration, tasks, robots, distance);

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

void TestController::Experiment3(uint32_t repetitions, uint32_t minT, uint32_t maxT, uint32_t stepT)
{
    std::cout << "Tasks"
              << "\t"
              << "Robots"
              << "\t"
              << "AvgTime"
              << "\t"
              << "stdTime"
              << "\t"
              << "HitRate"
              << "\t"
              << "IslandsHitRate" << std::endl;

    GAParameters gaP;
    gaP.populationSize = 500;
    gaP.mutationRate = 0.25;
    gaP.elitismRate = 0.05;
    gaP.maxIterations = 30;
    gaP.noChangeLimit = gaP.maxIterations * 0.5;

    for (uint32_t nt = minT; nt <= maxT; nt += stepT)
    {
        uint32_t nr = std::round(0.00190476 * nt * nt + 0.16190476 * nt + 2.14285714);
        gaP.populationSize = nt * 10;

        std::vector<double> islandHitRate(36, 0.0);
        double globalHitRate = 0.0;
        std::vector<int64_t> timeDuration(repetitions);

        std::cout << nt << "\t" << nr << "\t";

        for (uint32_t rep = 0; rep < repetitions; rep++)
        {
            lc->gerenateLocations(10, 3, 10, 50);
            auto d = lc->getDistanceMatrix();
            rc->generateRobots(nr, 1, 10, 0.5, 3, 1.0 / (8 * 60 * 60), 1.0 / (4 * 60 * 60), 3);
            auto robots = rc->getFreeRobots();
            tc->generateTasks(nt, robots, d, 3);
            auto tasks = tc->getTaskList();
            auto distance = lc->getDistanceMatrix();

            Island is(gaP, 50, 0.05, tasks, robots, distance); //verificar!!

            auto start = std::chrono::high_resolution_clock::now();
            is.setTimeLimit(10);
            is.solve();
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            
            timeDuration[rep] = duration.count();

            Chromosome best = is.getBest();
            globalHitRate += ((double)best.numberOfScheduled()/nt) / (double)repetitions;  
        }

        double avgTime = computeMean<int64_t>(timeDuration);
        double stdTime = std::sqrt(computeVariance<int64_t>(timeDuration, avgTime));

        std::cout << avgTime << "\t" << stdTime << "\t" << globalHitRate << std::endl;
    }
}