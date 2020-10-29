#ifndef LOCATION_CONTROLLER_HPP
#define LOCATION_CONTROLLER_HPP

#include "../model/location.hpp"
#include "../dao/locationDao.hpp"

#include <thread>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

class LocationController
{
private:
    LocationDao ld;
    std::vector<Location> locationList;
    std::vector<Location> depotList;
    std::vector<std::vector<double>> distanceMatrix;

    //Used to periodicly send server's postion transform
    ros::NodeHandle nh;
    ros::Publisher pubPosition;
    ros::Subscriber subPlan;
    std::thread *threadServerTransform;
    volatile double currX;
    volatile double currY;
    volatile double currA;
    volatile double goalX;
    volatile double goalY;
    volatile double goalA;
    volatile double pathLength;
    volatile bool planDone;
    //Sends the server position transform
    void sendServerTransform();
    //Sends the goal postion
    void sendGoalPosition();
    //Receavs the path and update distance matrix position
    void callbackPlanner(const nav_msgs::Path &msg);
    //Sends the server position transform periodicly to the global planner not fail
    void loopTransform();

public:
    LocationController();
    LocationController(GeneralDao *gDao);
    ~LocationController();

    void updateLocationList();
    void updateDistanceMatrix();

    Location *getLocationById(uint32_t id, bool isDepot = false);
    Location *getLocationByIndex(uint32_t index, bool isDepot = false);
    std::size_t getLocationListSize();
    std::size_t getDepotListSize();

    void setDistance(uint32_t from, uint32_t to, double distance);
    double getDistance(uint32_t from, uint32_t to);
    std::vector<std::vector<double>> *getDistanceMatrix();
    void gerenateLocations(uint32_t numOfLocations, uint32_t numOfDepots, double minDistance, double maxDistance);

    //Can be used only if locationList have all id;
    void computeDistanceMatrix(bool justMissingLocations);
    void run();
};

#endif