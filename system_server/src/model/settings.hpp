#ifndef SETTINGS_HPP
#define SETTINGS_HPP

#include <cstdint>

class Settings
{
private:
    uint32_t idSetup;
    uint32_t taskPoolSize;
    uint32_t robotPoolSize;
    uint32_t timeInterval;
    uint32_t gaIterations;
    uint32_t gaSubIterations;
    uint32_t gaPopulation;
    uint32_t gaTimeLimit;
    double gaNoChangeLimit;
    double gaElitism;
    double gaMutation;
    double gaMigration;

public:
    Settings(/* args */){}
    ~Settings(){}

    void setIdSetup(uint32_t idSetup);
    void setTaskPoolSize(uint32_t taskPoolSize);
    void setRobotPoolSize(uint32_t robotPoolSize);
    void setTimeInterval(uint32_t timeInterval);
    void setGaIterations(uint32_t gaIterations);
    void setGaSubIterations(uint32_t gaSubIterations);
    void setGaPopulation(uint32_t gaPopulation);
    void setGaTimeLimit(uint32_t gaTimeLimit);
    void setGaNoChangeLimit(uint32_t gaNoChangeLimit);
    void setGaElitism(double gaElitism);
    void setGaMutation(double gaMutation);
    void setGaMigration(double gaMigration);

    uint32_t getIdSetup() const;
    uint32_t getTaskPoolSize() const;
    uint32_t getRobotPoolSize() const;
    uint32_t getTimeInterval() const;
    uint32_t getGaIterations() const;
    uint32_t getGaSubIterations() const;
    uint32_t getGaPopulation() const;
    uint32_t getGaTimeLimit() const;
    double getGaNoChangeLimit() const;
    double getGaElitism() const;
    double getGaMutation() const;
    double getGaMigration() const;
};

#endif