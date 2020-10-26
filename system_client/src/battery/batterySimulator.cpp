#include "batterySimulator.hpp"

#include <random>

BatterySimulator::BatterySimulator()
{
    this->currBattery = 100;
    this->dischargeRate = 0.1;
    this->noise = 0;
    startTime = std::chrono::steady_clock::now();
}

BatterySimulator::BatterySimulator(double currBattery, double dischargeRate, double chargeRate, double noise)
{
    this->currBattery = currBattery;
    this->dischargeRate = dischargeRate;
    this->chargeRate = chargeRate;
    this->noise = noise;
    startTime = std::chrono::steady_clock::now();
}

BatterySimulator::~BatterySimulator() {}

void BatterySimulator::setCurrBattery(double currBattery)
{
    this->currBattery = currBattery;
}

void BatterySimulator::setDischargeRate(double dischargeRate)
{
    this->dischargeRate = dischargeRate;
}

void BatterySimulator::setChargeRate(double chargeRate)
{
    this->chargeRate = chargeRate;
}

void BatterySimulator::setNoise(double noise)
{
    this->noise = noise;
}

double BatterySimulator::getCurrBattery()
{
    return this->currBattery;
}

double BatterySimulator::getDischargeRate()
{
    return this->dischargeRate;
}

double BatterySimulator::getChargeRate()
{
    return this->chargeRate;
}

double BatterySimulator::getNoise()
{
    return this->noise;
}

void BatterySimulator::start()
{
    startTime = std::chrono::steady_clock::now();
}

double BatterySimulator::getRemaningBattery(bool charging)
{
    static std::random_device rd;
    static std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::normal_distribution<double> noiseDist(0.0, noise);

    std::chrono::steady_clock::time_point endTime = std::chrono::steady_clock::now();

    double elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() / 1000.0;
    elapsedTime += noiseDist(gen);

    if (charging)
        currBattery += elapsedTime * chargeRate;
    else
        currBattery -= elapsedTime * dischargeRate;

    if (currBattery < 5)
        currBattery = 5;
    else if (currBattery > 100)
        currBattery = 100;

    startTime = std::chrono::steady_clock::now();
    return currBattery;
}