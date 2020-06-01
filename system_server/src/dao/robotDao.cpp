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

bool RobotDao::getRobotList(std::vector<Robot> &robotList, std::string status)
{
    std::unique_ptr<sql::ResultSet> res;
    std::string stmt = "SELECT * FROM robot WHERE status = '" + status + "' ORDER BY id_robot ASC";

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

    return tst;
}

bool RobotDao::updateRobot(Robot &robot)
{
    std::ostringstream stmtStream;
    stmtStream << "UPDATE robot SET ";
    stmtStream << "description = '" << robot.getDescription() << "', ";
    stmtStream << "status = '" << robot.getStatus() << "', ";
    stmtStream << "max_payload = '" << robot.getMaximumPayload() << "', ";
    stmtStream << "remaining_battery = '" << robot.getRemainingBattery() << "', ";
    stmtStream << "discharge_factor = '" << robot.getDischargeFactor() << "', ";
    stmtStream << "battery_threshold = '" << robot.getBatteryThreshold() << "', ";
    stmtStream << "medium_velocity = '" << robot.getMediumVelocity() << "', ";
    stmtStream << "id_depot = '" << robot.getDepot() << "', ";
    stmtStream << "id_current_location = '" << robot.getCurrentLocation() << "' ";
    stmtStream << "WHERE robot.id_robot =" << robot.getId() << ";";

    bool tst = gDao->executeUpdate(stmtStream.str());

    return tst;
}

bool RobotDao::updateRobot(std::vector<Robot> &robotList)
{
    std::vector<std::string> statementVector;

    for (auto robot : robotList)
    {
        std::ostringstream stmtStream;
        stmtStream << "UPDATE robot SET ";
        stmtStream << "description = '" << robot.getDescription() << "', ";
        stmtStream << "status = '" << robot.getStatus() << "', ";
        stmtStream << "max_payload = '" << robot.getMaximumPayload() << "', ";
        stmtStream << "remaining_battery = '" << robot.getRemainingBattery() << "', ";
        stmtStream << "discharge_factor = '" << robot.getDischargeFactor() << "', ";
        stmtStream << "battery_threshold = '" << robot.getBatteryThreshold() << "', ";
        stmtStream << "medium_velocity = '" << robot.getMediumVelocity() << "', ";
        stmtStream << "id_depot = '" << robot.getDepot() << "', ";
        stmtStream << "id_current_location = '" << robot.getCurrentLocation() << "' ";
        stmtStream << "WHERE robot.id_robot =" << robot.getId() << ";";

        statementVector.push_back(stmtStream.str());
    }

    bool tst = gDao->executeUpdate(statementVector);

    return tst;
}

bool RobotDao::updateRobotRequest(RobotRequestData robotRequestData)
{
    std::ostringstream stmtStream;
    stmtStream << "UPDATE robot SET ";
    stmtStream << "status = '" << robotRequestData.status << "', ";
    stmtStream << "remaining_battery = '" << robotRequestData.remainingBattery << "', ";
    stmtStream << "medium_velocity = '" << robotRequestData.mediumVelocity << "', ";
    stmtStream << "id_current_location = '" << robotRequestData.currentLocation << "' ";
    stmtStream << "WHERE robot.id_robot =" << robotRequestData.id << ";";

    bool tst = gDao->executeUpdate(stmtStream.str());

    return tst;
}