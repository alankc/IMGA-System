#include "taskDao.hpp"

TaskDao::TaskDao(GeneralDao *gDao)
{
    this->gDao = gDao;
}

TaskDao::~TaskDao() {}

bool TaskDao::getTaskList(std::vector<Task> &taskList, std::string status)
{
    std::unique_ptr<sql::ResultSet> res;
    std::string stmt = "SELECT * FROM task WHERE status = '" + status + "' ORDER BY id_task ASC";

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