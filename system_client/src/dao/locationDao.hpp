#ifndef LOCATION_DAO_HPP
#define LOCATION_DAO_HPP

#include "generalDao.hpp"
#include "../model/location.hpp"

typedef struct _DistanceUnit
{
    uint32_t from;
    uint32_t to;
    double distance;
} DistanceUnit;

class LocationDao
{
private:
    GeneralDao *gDao;

public:
    LocationDao();
    LocationDao(GeneralDao *gDao);
    ~LocationDao();
    bool getLocationList(std::vector<Location> &locationList);
    bool getDistanceTable(std::vector<DistanceUnit> &distanceVector);
};

#endif