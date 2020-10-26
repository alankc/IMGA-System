#ifndef BATTERY_SIMULATOR_HPP
#define BATTERY_SIMULATOR_HPP

#include <chrono>
#include <ros/ros.h>

class BatterySimulator
{
private:
    double currBattery;
    double dischargeRate;
    double chargeRate;
    double noise;
    std::chrono::steady_clock::time_point startTime;

public:
    BatterySimulator();
    BatterySimulator(double currBattery, double dischargeRate, double chargeRate, double noise);
    ~BatterySimulator();

    void setCurrBattery(double currBattery);
    void setDischargeRate(double dischargeRate);
    void setChargeRate(double chargeRate);
    void setNoise(double noise);

    double getCurrBattery();
    double getDischargeRate();
    double getChargeRate();
    double getNoise();

    void start();

    double getRemaningBattery(bool charging = false);
};

#endif