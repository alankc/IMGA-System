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

bool RobotDao::getRobot(uint32_t id, Robot &r)
{
    std::unique_ptr<sql::ResultSet> res;
    std::string stmt = "SELECT * FROM robot WHERE id_robot = " + std::to_string(id) + ";";

    bool tst = gDao->executeQuery(stmt, res);

    if (tst)
    {
        res->next();

        r.setId(res->getUInt("id_robot"));
        r.setDepot(res->getUInt("id_depot"));
        r.setCurrentLocation(res->getUInt("id_current_location"));
        r.setMaximumPayload(res->getUInt("max_payload"));
        r.setRemainingBattery(res->getDouble("remaining_battery"));
        r.setDischargeFactor(res->getDouble("discharge_factor"));
        r.setBatteryThreshold(res->getDouble("battery_threshold"));
        r.setMediumVelocity(res->getDouble("medium_velocity"));
        r.setStatus(res->getString("status"));
        r.setDescription(res->getString("description"));
    }

    return tst;
}