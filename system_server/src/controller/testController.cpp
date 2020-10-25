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

        std::cout << "Tasks:" << nt << "\tRobots:" << nr << "\n";
        std::cout << "Iteration\t";
        std::cout << "timeToScheduling\t"
                  << "scheduledPercentage\t";
        std::cout << "fitnessExpected\t"
                  << "fitnessObtained\t";
        std::cout << "energyExpected\t"
                  << "energyObtained\t";
        std::cout << "timeExpected\t"
                  << "timeObtained\t";
        std::cout << "payloadExpected\t"
                  << "payloadObtained\t";
        std::cout << "meanNormEnergyE\t"
                  << "stdNormEnergyE\t";
        std::cout << "meanNormEnergyO\t"
                  << "stdNormEnergyO\t";
        std::cout << "meanNormPayloadE\t"
                  << "stdNormPayloadE\t";
        std::cout << "meanNormPayloadO\t"
                  << "stdNormPayloadO\t\tIslands" << std::endl;

        for (uint32_t rep = 0; rep < repetitions; rep++)
        {
            lc->gerenateLocations(10, 3, 10, 50);
            auto d = lc->getDistanceMatrix();
            rc->generateRobots(nr, 1, 10, 0.5, 3, 1.0 / (8 * 60 * 60), 1.0 / (4 * 60 * 60), 3);
            auto robots = rc->getFreeRobots();
            auto goalChrm = tc->generateTasks(nt, robots, d, 3);
            auto tasks = tc->getTaskList();
            auto distance = lc->getDistanceMatrix();

            Island is(gaP, 50, 0.05, tasks, robots, distance); //verificar!!

            auto start = std::chrono::high_resolution_clock::now();
            is.setTimeLimit(90);
            is.solve();
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

            Chromosome best = is.getBest();

            Exp3Result r;

            r.timeToScheduling = duration.count();
            r.scheduledPercentage = (double)best.numberOfScheduled() / (double)nt;

            r.energyExpected = goalChrm.totalEnergy(false);
            r.energyObtained = best.totalEnergy(false);

            r.timeExpected = goalChrm.totalTime(false);
            r.timeObtained = best.totalTime(false);

            r.payloadExpected = goalChrm.totalPayload(false);
            r.payloadObtained = best.totalPayload(false);

            auto energyVec = goalChrm.totalEnergyByRobot(true, false);
            auto payloadVec = goalChrm.totalPayloadByRobot(true, false);

            r.meanNormEnergyE = computeMean<double>(energyVec);
            r.stdNormEnergyE = std::sqrt(computeVariance<double>(energyVec, r.meanNormEnergyE));

            r.meanNormPayloadE = computeMean<double>(payloadVec);
            r.stdNormPayloadE = std::sqrt(computeVariance<double>(payloadVec, r.meanNormPayloadE));

            energyVec = best.totalEnergyByRobot(true, false);
            payloadVec = best.totalPayloadByRobot(true, false);

            r.meanNormEnergyO = computeMean<double>(energyVec);
            r.stdNormEnergyO = std::sqrt(computeVariance<double>(energyVec, r.meanNormEnergyO));

            r.meanNormPayloadO = computeMean<double>(payloadVec);
            r.stdNormPayloadO = std::sqrt(computeVariance<double>(payloadVec, r.meanNormPayloadO));

            std::cout << rep << "\t";
            std::cout << std::fixed << r.timeToScheduling << "\t" << r.scheduledPercentage << "\t";
            std::cout << std::fixed << goalChrm.getFitness() << "\t" << best.getFitness() << "\t";
            std::cout << std::fixed << r.energyExpected << "\t" << r.energyObtained << "\t";
            std::cout << std::fixed << r.timeExpected << "\t" << r.timeObtained << "\t";
            std::cout << std::fixed << r.payloadExpected << "\t" << r.payloadObtained << "\t";
            std::cout << std::fixed << r.meanNormEnergyE << "\t" << r.stdNormEnergyE << "\t";
            std::cout << std::fixed << r.meanNormEnergyO << "\t" << r.stdNormEnergyO << "\t";
            std::cout << std::fixed << r.meanNormPayloadE << "\t" << r.stdNormPayloadE << "\t";
            std::cout << std::fixed << r.meanNormPayloadO << "\t" << r.stdNormPayloadO << "\t";

            auto bestList = is.getListOfBests();
            for (uint32_t j = 0; j < bestList.size(); j++)
            {
                if (bestList[j].allScheduled())
                    std::cout << "\t" << 1;
                else
                    std::cout << "\t" << 0;
            }

            std::cout << std::endl;
        }

        // std::cout << avgTime << "\t" << stdTime << "\t" << globalHitRate << std::endl;
    }
}

void TestController::ExperimentGAPure(uint32_t repetitions, uint32_t minT, uint32_t maxT, uint32_t stepT)
{
    GAParameters gaP;
    gaP.populationSize = 500;
    gaP.mutationRate = 0.25;
    gaP.elitismRate = 0.05;
    gaP.maxIterations = 1500;
    gaP.noChangeLimit = gaP.maxIterations * 0.50;
    gaP.crossoverMode = GA::CrossoverMode::positionBased;
    gaP.selectionMode = GA::SelectionMode::tournament;
    gaP.mutationMode = GA::MutationMode::exchange;

    for (uint32_t nt = minT; nt <= maxT; nt += stepT)
    {
        uint32_t nr = std::round(0.00190476 * nt * nt + 0.16190476 * nt + 2.14285714);
        gaP.populationSize = nt * 10;

        std::vector<int64_t> timeDuration(repetitions);

        std::cout << "Tasks:" << nt << "\tRobots:" << nr << "\n";
        std::cout << "Iteration\t";
        std::cout << "timeToScheduling\t"
                  << "scheduledPercentage\t";
        std::cout << "fitnessExpected\t"
                  << "fitnessObtained\t";
        std::cout << "energyExpected\t"
                  << "energyObtained\t";
        std::cout << "timeExpected\t"
                  << "timeObtained\t";
        std::cout << "payloadExpected\t"
                  << "payloadObtained\t";
        std::cout << "meanNormEnergyE\t"
                  << "stdNormEnergyE\t";
        std::cout << "meanNormEnergyO\t"
                  << "stdNormEnergyO\t";
        std::cout << "meanNormPayloadE\t"
                  << "stdNormPayloadE\t";
        std::cout << "meanNormPayloadO\t"
                  << "stdNormPayloadO" << std::endl;

        for (uint32_t rep = 0; rep < repetitions; rep++)
        {
            lc->gerenateLocations(10, 3, 10, 50);
            auto d = lc->getDistanceMatrix();
            rc->generateRobots(nr, 1, 10, 0.5, 3, 1.0 / (8 * 60 * 60), 1.0 / (4 * 60 * 60), 3);
            auto robots = rc->getFreeRobots();
            auto goalChrm = tc->generateTasks(nt, robots, d, 3);
            auto tasks = tc->getTaskList();
            auto distance = lc->getDistanceMatrix();

            GA ga(gaP); //verificar!!
            Chromosome::setRobotList(robots);
            Chromosome::setTaskList(tasks);
            Chromosome::setDistanceMatrix(distance);
            ga.initialize();

            auto start = std::chrono::high_resolution_clock::now();
            ga.setTimeLimit(90);
            ga.solve();
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

            Chromosome best = ga.getBest();

            Exp3Result r;

            r.timeToScheduling = duration.count();
            r.scheduledPercentage = (double)best.numberOfScheduled() / (double)nt;

            r.energyExpected = goalChrm.totalEnergy(false);
            r.energyObtained = best.totalEnergy(false);

            r.timeExpected = goalChrm.totalTime(false);
            r.timeObtained = best.totalTime(false);

            r.payloadExpected = goalChrm.totalPayload(false);
            r.payloadObtained = best.totalPayload(false);

            auto energyVec = goalChrm.totalEnergyByRobot(true, false);
            auto payloadVec = goalChrm.totalPayloadByRobot(true, false);

            r.meanNormEnergyE = computeMean<double>(energyVec);
            r.stdNormEnergyE = std::sqrt(computeVariance<double>(energyVec, r.meanNormEnergyE));

            r.meanNormPayloadE = computeMean<double>(payloadVec);
            r.stdNormPayloadE = std::sqrt(computeVariance<double>(payloadVec, r.meanNormPayloadE));

            energyVec = best.totalEnergyByRobot(true, false);
            payloadVec = best.totalPayloadByRobot(true, false);

            r.meanNormEnergyO = computeMean<double>(energyVec);
            r.stdNormEnergyO = std::sqrt(computeVariance<double>(energyVec, r.meanNormEnergyO));

            r.meanNormPayloadO = computeMean<double>(payloadVec);
            r.stdNormPayloadO = std::sqrt(computeVariance<double>(payloadVec, r.meanNormPayloadO));

            std::cout << rep << "\t";
            std::cout << std::fixed << r.timeToScheduling << "\t" << r.scheduledPercentage << "\t";
            std::cout << std::fixed << goalChrm.getFitness() << "\t" << best.getFitness() << "\t";
            std::cout << std::fixed << r.energyExpected << "\t" << r.energyObtained << "\t";
            std::cout << std::fixed << r.timeExpected << "\t" << r.timeObtained << "\t";
            std::cout << std::fixed << r.payloadExpected << "\t" << r.payloadObtained << "\t";
            std::cout << std::fixed << r.meanNormEnergyE << "\t" << r.stdNormEnergyE << "\t";
            std::cout << std::fixed << r.meanNormEnergyO << "\t" << r.stdNormEnergyO << "\t";
            std::cout << std::fixed << r.meanNormPayloadE << "\t" << r.stdNormPayloadE << "\t";
            std::cout << std::fixed << r.meanNormPayloadO << "\t" << r.stdNormPayloadO << std::endl;
        }

        // std::cout << avgTime << "\t" << stdTime << "\t" << globalHitRate << std::endl;
    }
}