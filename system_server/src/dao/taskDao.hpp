#ifndef TASK_DAO_HPP
#define TASK_DAO_HPP

#include "generalDao.hpp"
#include "../model/task.hpp"

typedef struct _taskScheduledData
{
	uint32_t id;
	std::string status;
	uint32_t robotInCharge;
	uint32_t seqNumber;
}TaskScheduledData;


class TaskDao
{
private:
    GeneralDao *gDao;

public:
    TaskDao(GeneralDao *gDao);
    ~TaskDao();
    bool getTaskList(std::vector<Task> &taskList, std::string status);
    bool updateTask(Task &task);
    bool updateTask(std::vector<Task> &taskList);
    bool updateTasksScheduled(std::vector<TaskScheduledData> &taskList);
};

#endif