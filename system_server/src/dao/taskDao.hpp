#ifndef TASK_DAO_HPP
#define TASK_DAO_HPP

#include "generalDao.hpp"
#include "../model/task.hpp"

class TaskDao
{
private:
    GeneralDao *gDao;

public:
    TaskDao(GeneralDao *gDao);
    ~TaskDao();
    bool getTaskList(std::vector<Task> &taskList, std::string status);
};

#endif