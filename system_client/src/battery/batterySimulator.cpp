#include "batterySimulator.hpp"

#include <random>

BatterySimulator::BatterySimulator()
{
    this->startBattery = 100;
    this->dischargeRate = 0.1;
    this->noise = 0;
    startTime = std::chrono::steady_clock::now();
}

BatterySimulator::BatterySimulator(double startBattery, double dischargeRate, double noise)
{
    this->startBattery = startBattery;
    this->dischargeRate = dischargeRate;
    this->noise = noise;
    startTime = std::chrono::steady_clock::now();
}

BatterySimulator::~BatterySimulator() {}

void BatterySimulator::setStartBattery(double startBattery)
{
    this->startBattery = startBattery;
}

void BatterySimulator::setDischargeRate(double dischargeRate)
{
    this->dischargeRate = dischargeRate;
}

void BatterySimulator::setNoise(double noise)
{
    this->noise = noise;
}

double BatterySimulator::getStartBattery()
{
    return this->startBattery;
}

double BatterySimulator::getDischargeRate()
{
    return this->dischargeRate;
}

double BatterySimulator::getNoise()
{
    return this->noise;
}

void BatterySimulator::start()
{
    startTime = std::chrono::steady_clock::now();
}

double BatterySimulator::getRemaningBattery()
{
    static std::random_device rd;
    static std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::normal_distribution<double> noiseDist(0.0, noise);

    std::chrono::steady_clock::time_point endTime = std::chrono::steady_clock::now();

    double elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() / 1000.0;
    elapsedTime += noiseDist(gen);

    return startBattery - elapsedTime * dischargeRate;
}