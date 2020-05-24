#include "distanceTableDao.hpp"

DistanceTableDao::DistanceTableDao(GeneralDao *gDao)
{
    this->gDao = gDao;
}

DistanceTableDao::~DistanceTableDao(){}


bool DistanceTableDao::getDistanceTable(std::vector<DistanceUnit> &distanceVector)
{
    std::unique_ptr<sql::ResultSet> res;
    std::string stmt = "SELECT * FROM distance_table ORDER BY id_location_from ASC";

    bool tst = gDao->executeQuery(stmt, res);

    if (tst)
    {
        while (res->next())
        {
            DistanceUnit tmp;
            tmp.from = res->getUInt("id_location_from");
            tmp.to = res->getUInt("id_location_to");
            tmp.distance = res->getDouble("distance");

            distanceVector.push_back(tmp);
        }
    }

    return tst;
}