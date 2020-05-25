#ifndef LOCATION_DAO_HPP
#define LOCATION_DAO_HPP

#include "generalDao.hpp"
#include "../model/location.hpp"

class LocationDao
{
private:
    GeneralDao *gDao;

public:
    LocationDao(GeneralDao *gDao);
    ~LocationDao();
    bool getLocationList(std::vector<Location> &locationList);
};

#endif