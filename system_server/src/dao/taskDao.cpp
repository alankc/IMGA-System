#include "taskDao.hpp"

#include <sstream>

TaskDao::TaskDao() {}

TaskDao::TaskDao(GeneralDao *gDao)
{
    this->gDao = gDao;
}

TaskDao::~TaskDao() {}

void TaskDao::setGeneralDao(GeneralDao *gDao)
{
    this->gDao = gDao;
}

bool TaskDao::getTaskList(std::vector<Task> &taskList, std::string status, uint32_t numberOfTasks)
{
    std::unique_ptr<sql::ResultSet> res;
    std::string stmt = "SELECT * FROM task WHERE status = '" + status + "' ORDER BY id_task ASC LIMIT " + std::to_string(numberOfTasks) + ";";

    bool tst = gDao->executeQuery(stmt, res);

    if (tst)
    {
        while (res->next())
        {
            Task tmp;
            tmp.setId(res->getUInt("id_task"));
            tmp.setPickUpLocation(res->getUInt("id_pick_up_location"));
            tmp.setDeliveryLocation(res->getUInt("id_delivery_location"));
            tmp.setPayload(res->getUInt("payload"));
            tmp.setDeadline(res->getUInt("deadline"));
            tmp.setStatus(res->getString("status"));
            tmp.setDescription(res->getString("description"));

            if (!res->isNull("id_robot_in_charge"))
                tmp.setRobotInCharge(res->getUInt("id_robot_in_charge"));
            if (!res->isNull("seq_number"))
                tmp.setSeqNumber(res->getUInt("seq_number"));
            if (!res->isNull("start_time"))
                tmp.setStartTime(res->getDouble("start_time"));
            if (!res->isNull("end_time"))
                tmp.setEndTime(res->getDouble("end_time"));

            taskList.push_back(tmp);
        }
    }

    return tst;
}

bool TaskDao::getTaskIdList(std::vector<uint32_t> &taskList, std::string status, uint32_t numberOfTasks)
{
    std::unique_ptr<sql::ResultSet> res;
    std::string stmt = "SELECT id_task FROM task WHERE status = '" + status + "' ORDER BY id_task ASC LIMIT " + std::to_string(numberOfTasks) + ";";

    bool tst = gDao->executeQuery(stmt, res);

    if (tst)
    {
        while (res->next())
        {
            taskList.push_back(res->getUInt("id_task"));
        }
    }

    return tst;
}

bool TaskDao::getMaximumSeqNum(uint32_t idRobot, int64_t &maxSeNumber)
{
    std::unique_ptr<sql::ResultSet> res;
    std::string stmt = "SELECT MAX(task.seq_number) AS MAX_SEQ_NUMBER\n";
    stmt += "FROM task\n";
    stmt += "INNER JOIN robot\n";
    stmt += "ON task.id_robot_in_charge = robot.id_robot\n";
    stmt += "WHERE robot.id_robot = " + std::to_string(idRobot) + ";";

    bool tst = gDao->executeQuery(stmt, res);

    if (tst)
    {
        if (res->next())
        {
            if (res->isNull("MAX_SEQ_NUMBER"))
                maxSeNumber = -1;
            else
                maxSeNumber = res->getUInt("MAX_SEQ_NUMBER");
        }
        else
            return false;
    }

    return tst;
}

bool TaskDao::updateTask(Task &task)
{
    std::ostringstream stmtStream;
    stmtStream << "UPDATE task SET ";
    stmtStream << "description = '" << task.getDescription() << "', ";
    stmtStream << "status = '" << task.getStatus() << "', ";
    stmtStream << "payload = '" << task.getPayload() << "', ";
    stmtStream << "deadline = '" << task.getDeadline() << "', ";
    stmtStream << "id_pick_up_location = '" << task.getPickUpLocation() << "', ";
    stmtStream << "id_delivery_location = '" << task.getDeliveryLocation() << "', ";
    stmtStream << "id_robot_in_charge = '" << task.getRobotInCharge() << "', ";
    stmtStream << "seq_number = '" << task.getSeqNumber() << "', ";
    stmtStream << "start_time = '" << task.getStartTime() << "', ";
    stmtStream << "end_time = '" << task.getEndTime() << "' ";
    stmtStream << "WHERE task.id_task = " << task.getId() << ";";

    bool tst = gDao->executeUpdate(stmtStream.str());

    return tst;
}

bool TaskDao::updateTask(std::vector<Task> &taskList)
{
    std::vector<std::string> statementVector;

    for (auto task : taskList)
    {
        std::ostringstream stmtStream;
        stmtStream << "UPDATE task SET ";
        stmtStream << "description = '" << task.getDescription() << "', ";
        stmtStream << "status = '" << task.getStatus() << "', ";
        stmtStream << "payload = '" << task.getPayload() << "', ";
        stmtStream << "deadline = '" << task.getDeadline() << "', ";
        stmtStream << "id_pick_up_location = '" << task.getPickUpLocation() << "', ";
        stmtStream << "id_delivery_location = '" << task.getDeliveryLocation() << "', ";
        stmtStream << "id_robot_in_charge = '" << task.getRobotInCharge() << "', ";
        stmtStream << "seq_number = '" << task.getSeqNumber() << "', ";
        stmtStream << "start_time = '" << task.getStartTime() << "', ";
        stmtStream << "end_time = '" << task.getEndTime() << "' ";
        stmtStream << "WHERE task.id_task = " << task.getId() << ";";

        statementVector.push_back(stmtStream.str());
    }

    bool tst = gDao->executeUpdate(statementVector);

    return tst;
}

bool TaskDao::updateTasksScheduled(std::vector<TaskScheduledData> &taskList)
{
    std::vector<std::string> statementVector;

    for (auto task : taskList)
    {
        std::ostringstream stmtStream;
        if (task.status == Task::STATUS_SCHEDULED)
        {
            stmtStream << "UPDATE task SET ";
            stmtStream << "status = '" << task.status << "', ";
            stmtStream << "id_robot_in_charge = '" << task.robotInCharge << "', ";
            stmtStream << "seq_number = '" << task.seqNumber << "' ";
            stmtStream << "WHERE task.id_task = " << task.id << ";";
        }
        else
        {
            stmtStream << "UPDATE task SET ";
            stmtStream << "status = '" << task.status << "' ";
            stmtStream << "WHERE task.id_task = " << task.id << ";";
        }

        statementVector.push_back(stmtStream.str());
    }

    bool tst = gDao->executeUpdate(statementVector);

    return tst;
}

bool TaskDao::updateTask(uint32_t id, TaskDao::Column column, std::string data)
{
    std::ostringstream stmtStream;
    stmtStream << "UPDATE task SET ";
    switch (column)
    {
    case (TaskDao::Column::description):
        stmtStream << "description = '" << data << "'";
        break;
    case (TaskDao::Column::status):
        stmtStream << "status = '" << data << "'";
        break;
    case (TaskDao::Column::pickUpLocation):
        stmtStream << "id_pick_up_location = " << data;
        break;
    case (TaskDao::Column::deliveryLocation):
        stmtStream << "id_delivery_location = " << data;
        break;
    case (TaskDao::Column::payload):
        stmtStream << "payload = " << data;
        break;
    case (TaskDao::Column::deadline):
        stmtStream << "deadline = " << data;
        break;
    case (TaskDao::Column::robotInCharge):
        stmtStream << "id_robot_in_charge = " << data;
        break;
    case (TaskDao::Column::seqNumber):
        stmtStream << "seq_number = " << data;
        break;
    case (TaskDao::Column::startTime):
        stmtStream << "start_time = " << data;
        break;
    case (TaskDao::Column::endTime):
        stmtStream << "end_time = " << data;
        break;

    default:
        return false;
    }

    stmtStream << " WHERE task.id_task = " << id << ";";

    bool tst = gDao->executeUpdate(stmtStream.str());

    return tst;
}