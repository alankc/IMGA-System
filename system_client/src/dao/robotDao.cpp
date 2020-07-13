#include "robotDao.hpp"

#include <sstream>

RobotDao::RobotDao() {}

RobotDao::RobotDao(GeneralDao *gDao)
{
    this->gDao = gDao;
}

RobotDao::~RobotDao() {}

void RobotDao::setGeneralDao(GeneralDao *gDao)
{
    this->gDao = gDao;
}

Robot RobotDao::getRobot(uint32_t id)
{
    /*std::unique_ptr<sql::ResultSet> res;
    std::string stmt = "SELECT * FROM robot WHERE status LIKE '%" + status + "%' ORDER BY id_robot ASC";

    bool tst = gDao->executeQuery(stmt, res);

    if (tst)
    {
        while (res->next())
        {
            Robot tmp;
            tmp.setId(res->getUInt("id_robot"));
            tmp.setDepot(res->getUInt("id_depot"));
            tmp.setCurrentLocation(res->getUInt("id_current_location"));
            tmp.setMaximumPayload(res->getUInt("max_payload"));
            tmp.setRemainingBattery(res->getDouble("remaining_battery"));
            tmp.setDischargeFactor(res->getDouble("discharge_factor"));
            tmp.setBatteryThreshold(res->getDouble("battery_threshold"));
            tmp.setMediumVelocity(res->getDouble("medium_velocity"));
            tmp.setStatus(res->getString("status"));
            tmp.setDescription(res->getString("description"));

            robotList.push_back(tmp);
        }
    }

    return tst;*/
}