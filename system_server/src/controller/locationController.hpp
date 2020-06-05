#ifndef LOCATION_CONTROLLER_HPP
#define LOCATION_CONTROLLER_HPP

#include "../model/location.hpp"
#include "../dao/locationDao.hpp"

class LocationController
{
private:
    LocationDao ld;
    std::vector<Location> locationList;
    std::vector<Location> depotList;
    std::vector<std::vector<double>> distanceMatrix;

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
};

#endif