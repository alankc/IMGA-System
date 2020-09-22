#include "island.hpp"

#include <thread>
#include <iostream>
#include <cmath>
#include <random>
#include <algorithm>
#include <chrono>

Island::Island(GAParameters gaP, uint64_t maxSubIteration, double migrationRate, std::vector<Task> *taskList, std::vector<Robot> *robotList, std::vector<std::vector<double>> *distancematrix)
{
    this->gaP = gaP;
    this->maxSubIteration = maxSubIteration;
    this->migrationRate = migrationRate;
    Chromosome::setRobotList(robotList);
    Chromosome::setTaskList(taskList);
    Chromosome::setDistanceMatrix(distancematrix);
    this->seconds = 0;
}

Island::~Island()
{
}

void Island::migration(uint16_t ga1, uint16_t ga2)
{
    uint16_t migrationMaxCount = std::round(gaP.populationSize * migrationRate);

    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<uint8_t> migDist(0, 1);

    std::vector<Chromosome> &popGA1 = gaList[ga1]->getPopulation();
    std::vector<Chromosome> &popGA2 = gaList[ga2]->getPopulation();
    std::vector<bool> tst(gaP.populationSize, false); //tst if have alredy changed

    uint16_t i = 0;
    while (migrationMaxCount != 0)
    {
        if (!tst[i] && migDist(gen))
        {
            if (popGA1[i].getFitness() < popGA2[i].getFitness())
                popGA2[i] = popGA1[i];
            else
                popGA1[i] = popGA2[i];
            //std::swap(popGA1[i], popGA2[i]);
            migrationMaxCount--;
            tst[i] = true;
        }
        i = (i + 1) % gaP.populationSize;
    }
    std::sort(popGA1.begin(), popGA1.end());
    std::sort(popGA2.begin(), popGA2.end());
}

void Island::globalMigration()
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::vector<uint16_t> gaMigList(gaList.size());
    std::iota(gaMigList.begin(), gaMigList.end(), 0);
    std::shuffle(gaMigList.begin(), gaMigList.end(), gen);

    uint16_t i = 0, j = 1;
    uint16_t maxK = std::round(gaMigList.size() / 2.0);
    for (uint16_t k = 0; k < maxK; k++)
    {
        migration(gaMigList[i], gaMigList[j]);
        i = (i + 2) % gaMigList.size();
        j = (j + 2) % gaMigList.size();
    }
}

void Island::solve()
{

    auto start = std::chrono::high_resolution_clock::now();

    GAParameters gaTmp = gaP;
    gaTmp.maxIterations = maxSubIteration;
    gaTmp.noChangeLimit = ((double)gaTmp.noChangeLimit / gaTmp.maxIterations) * maxSubIteration;
    gaList.reserve(36);

    for (uint16_t i = 0; i < 3; i++)
    {
        for (uint16_t j = 0; j < 3; j++)
        {
            for (uint16_t k = 0; k < 4; k++)
            {
                gaTmp.selectionMode = static_cast<GA::SelectionMode>(i);
                gaTmp.crossoverMode = static_cast<GA::CrossoverMode>(j);
                gaTmp.mutationMode = static_cast<GA::MutationMode>(k);
                GA *ga = new GA(gaTmp);
                ga->initialize();
                gaList.push_back(ga);
            }
        }
    }

    std::vector<std::thread *> threadList;
    for (uint64_t i = 0; i < gaP.maxIterations; i++)
    {
        for (uint16_t j = 0; j < gaList.size(); j++)
        {
            auto &ga = gaList[j];
            std::thread *t = new std::thread(&GA::solve, ga);
            threadList.push_back(t);
        }

        uint16_t j = 0;
        bool breakTst = false;
        for (auto &t : threadList)
        {
            t->join();
            delete (t);

            auto &ga = gaList[j++];
            if (ga->getBest().allScheduled())
                breakTst = true;
        }

        threadList.clear();

        if (breakTst)
            break;

        if (seconds > 0)
        {
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            if (duration.count() >= seconds * 1000.0)
                break;
        }

        globalMigration();
    }

    best = gaList[0]->getBest();
    listOfBests.clear();
    listOfBests.push_back(best);
    for (uint16_t i = 1; i < gaList.size(); i++)
    {
        Chromosome tmp = gaList[i]->getBest();
        listOfBests.push_back(tmp);
        if (best.getFitness() > tmp.getFitness())
            best = tmp;
        delete (gaList[i]);
    }
    gaList.clear();
    //best.printResult();
}

Chromosome Island::getBest()
{
    return best;
}

std::vector<Chromosome> Island::getListOfBests()
{
    return listOfBests;
}

void Island::setTimeLimit(uint32_t seconds)
{
    this->seconds = seconds;
}