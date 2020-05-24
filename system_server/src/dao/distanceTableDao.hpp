#ifndef DISTANCE_TABLE_DAO_HPP
#define DISTANCE_TABLE_DAO_HPP

#include "generalDao.hpp"

typedef struct _DistanceUnit
{
    uint32_t from;
    uint32_t to;
    double distance;
} DistanceUnit;


class DistanceTableDao
{
private:
    GeneralDao *gDao;

public:

    DistanceTableDao(GeneralDao *gDao);
    ~DistanceTableDao();
    bool getDistanceTable(std::vector<DistanceUnit> &distanceVector);
};


#endif