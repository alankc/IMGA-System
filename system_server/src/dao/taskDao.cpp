#include "taskDao.hpp"

#include <sstream>

TaskDao::TaskDao(){}

TaskDao::TaskDao(GeneralDao *gDao)
{
    this->gDao = gDao;
}

TaskDao::~TaskDao() {}

void TaskDao::setGeneralDao(GeneralDao *gDao)
{
    this->gDao = gDao;
}

bool TaskDao::getTaskList(std::vector<Task> &taskList, std::string status)
{
    std::unique_ptr<sql::ResultSet> res;
    std::string stmt = "SELECT * FROM task WHERE status LIKE '%" + status + "%' ORDER BY id_task ASC";

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
            tmp.setRobotInCharge(res->getUInt("id_robot_in_charge"));
            tmp.setSeqNumber(res->getUInt("seq_number"));

            taskList.push_back(tmp);
        }
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
    stmtStream << "seq_number = '" << task.getSeqNumber() << "' ";
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
        stmtStream << "seq_number = '" << task.getSeqNumber() << "' ";
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
        stmtStream << "UPDATE task SET ";
        stmtStream << "status = '" << task.status << "', ";
        stmtStream << "id_robot_in_charge = '" << task.robotInCharge << "', ";
        stmtStream << "seq_number = '" << task.seqNumber << "' ";
        stmtStream << "WHERE task.id_task = " << task.id << ";";

        statementVector.push_back(stmtStream.str());
    }

    bool tst = gDao->executeUpdate(statementVector);

    return tst;
}