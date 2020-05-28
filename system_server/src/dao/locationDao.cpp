#include "locationDao.hpp"

LocationDao::LocationDao(GeneralDao *gDao)
{
    this->gDao = gDao;
}

LocationDao::~LocationDao() {}

bool LocationDao::getLocationList(std::vector<Location> &locationList)
{
    std::unique_ptr<sql::ResultSet> res;
    std::string stmt = "SELECT * FROM location ORDER BY id_location ASC";

    bool tst = gDao->executeQuery(stmt, res);

    if (tst)
    {
        while (res->next())
        {
            Location tmp;
            tmp.setId(res->getUInt("id_location"));
            tmp.setX(res->getUInt("x_pos"));
            tmp.setY(res->getUInt("y_pos"));
            tmp.setA(res->getUInt("a_pos"));
            tmp.setDescription(res->getString("description"));
            tmp.setIsDepot(res->getBoolean("is_depot"));

            locationList.push_back(tmp);
        }
    }

    return tst;
}