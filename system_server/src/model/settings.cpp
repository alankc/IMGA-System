#include "settings.hpp"

void Settings::setIdSetup(uint32_t idSetup)
{
    this->idSetup = idSetup;
}

void Settings::setTaskPoolSize(uint32_t taskPoolSize)
{
    this->taskPoolSize = taskPoolSize;
}

void Settings::setRobotPoolSize(uint32_t robotPoolSize)
{
    this->robotPoolSize = robotPoolSize;
}

void Settings::setTimeInterval(uint32_t timeInterval)
{
    this->timeInterval = timeInterval;
}

void Settings::setGaIterations(uint32_t gaIterations)
{
    this->gaIterations = gaIterations;
}

void Settings::setGaSubIterations(uint32_t gaSubIterations)
{
    this->gaSubIterations = gaSubIterations;
}

void Settings::setGaPopulation(uint32_t gaPopulation)
{
    this->gaPopulation = gaPopulation;
}

void Settings::setGaTimeLimit(uint32_t gaTimeLimit)
{
    this->gaTimeLimit = gaTimeLimit;
}

void Settings::setGaNoChangeLimit(uint32_t gaNoChangeLimit)
{
    this->gaNoChangeLimit = gaNoChangeLimit;
}

void Settings::setGaElitism(double gaElitism)
{
    this->gaElitism = gaElitism;
}

void Settings::setGaMutation(double gaMutation)
{
    this->gaMutation = gaMutation;
}

void Settings::setGaMigration(double gaMigration)
{
    this->gaMigration = gaMigration;
}

uint32_t Settings::getIdSetup() const
{
    return idSetup;
}

uint32_t Settings::getTaskPoolSize() const
{
    return taskPoolSize;
}

uint32_t Settings::getRobotPoolSize() const
{
    return robotPoolSize;
}

uint32_t Settings::getTimeInterval() const
{
    return timeInterval;
}

uint32_t Settings::getGaIterations() const
{
    return gaIterations;
}

uint32_t Settings::getGaSubIterations() const
{
    return gaSubIterations;
}

uint32_t Settings::getGaPopulation() const
{
    return gaPopulation;
}

uint32_t Settings::getGaTimeLimit() const
{
    return gaTimeLimit;
}

double Settings::getGaNoChangeLimit() const
{
    return gaNoChangeLimit;
}

double Settings::getGaElitism() const
{
    return gaElitism;
}

double Settings::getGaMutation() const
{
    return gaMutation;
}

double Settings::getGaMigration() const
{
    return gaMigration;
}