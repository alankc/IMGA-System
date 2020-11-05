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

bool RobotDao::getRobotList(std::vector<Robot> &robotList, bool equalToStatus, std::string status)
{
    std::unique_ptr<sql::ResultSet> res;
    std::string stmt;
    if (equalToStatus)
        stmt = "SELECT * FROM robot WHERE status = '" + status + "' ORDER BY id_robot ASC";
    else
        stmt = "SELECT * FROM robot WHERE status <> '" + status + "' ORDER BY id_robot ASC";

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
            tmp.setWaitingTime(res->getUInt("waiting_time"));

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
    stmtStream << "waiting_time = '" << robot.getWaitingTime() << "' ";
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
        stmtStream << "waiting_time = '" << robot.getWaitingTime() << "' ";
        stmtStream << "WHERE robot.id_robot =" << robot.getId() << ";";

        statementVector.push_back(stmtStream.str());
    }

    bool tst = gDao->executeUpdate(statementVector);

    return tst;
}

bool RobotDao::updateRobotRequest(system_server::MsgRobotData &robotRequestData)
{
    std::ostringstream stmtStream;
    stmtStream << "UPDATE robot SET ";
    stmtStream << "status = '" << robotRequestData.status << "', ";
    stmtStream << "remaining_battery = '" << robotRequestData.battery << "', ";
    stmtStream << "medium_velocity = '" << robotRequestData.minSpeed << "', ";
    stmtStream << "id_current_location = '" << robotRequestData.currLocation << "' ";
    stmtStream << "WHERE robot.id_robot =" << robotRequestData.id << ";";

    bool tst = gDao->executeUpdate(stmtStream.str());

    return tst;
}

bool RobotDao::updateRobot(uint32_t id, RobotDao::Column column, std::string data)
{
    std::ostringstream stmtStream;
    stmtStream << "UPDATE robot SET ";
    switch (column)
    {
    case (RobotDao::Column::description):
        stmtStream << "description = '" << data << "'";
        break;

    case (RobotDao::Column::status):
        stmtStream << "status = '" << data << "'";
        break;

    case (RobotDao::Column::maximumPayload):
        stmtStream << "max_payload = " << data;
        break;
    case (RobotDao::Column::remainingBattery):
        stmtStream << "remaining_battery = " << data;
        break;

    case (RobotDao::Column::dischargeFactor):
        stmtStream << "discharge_factor = " << data;
        break;

    case (RobotDao::Column::batteryThreshold):
        stmtStream << "battery_threshold = " << data;
        break;

    case (RobotDao::Column::mediumVelocity):
        stmtStream << "medium_velocity = " << data;
        break;

    case (RobotDao::Column::waitingTime):
        stmtStream << "waiting_time = " << data;
        break;

    case (RobotDao::Column::depot):
        stmtStream << "id_depot = " << data;
        break;

    case (RobotDao::Column::currentLocation):
        stmtStream << "id_current_location = " << data;
        break;

    default:
        return false;
    }

    stmtStream << " WHERE robot.id_robot = " << id << ";";

    bool tst = gDao->executeUpdate(stmtStream.str());

    return tst;
}