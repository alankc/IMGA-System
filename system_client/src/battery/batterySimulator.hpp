#ifndef BATTERY_SIMULATOR_HPP
#define BATTERY_SIMULATOR_HPP

#include <chrono>
#include <ros/ros.h>

class BatterySimulator
{
private:
    double startBattery;
    double dischargeRate;
    double noise;
    std::chrono::steady_clock::time_point startTime;

public:
    BatterySimulator();
    BatterySimulator(double startBattery, double dischargeRate, double noise);
    ~BatterySimulator();

    void setStartBattery(double startBattery);
    void setDischargeRate(double dischargeRate);
    void setNoise(double noise);

    double getStartBattery();
    double getDischargeRate();
    double getNoise();

    void start();

    double getRemaningBattery();
};

#endif