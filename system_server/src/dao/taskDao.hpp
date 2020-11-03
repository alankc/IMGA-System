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
} TaskScheduledData;

class TaskDao
{
private:
    GeneralDao *gDao;

public:
    TaskDao();
    TaskDao(GeneralDao *gDao);
    ~TaskDao();
    void setGeneralDao(GeneralDao *gDao);
    bool getTaskList(std::vector<Task> &taskList, std::string status, uint32_t numberOfTasks);
    bool getMaximumSeqNum(uint32_t idRobot, int64_t &maxSeNumber);
    bool updateTask(Task &task);
    bool updateTask(std::vector<Task> &taskList);
    bool updateTasksScheduled(std::vector<TaskScheduledData> &taskList);

    enum Column
    {
        description = 0,
        status = 1,
        pickUpLocation = 2,
        deliveryLocation = 3,
        payload = 4,
        deadline = 5,
        robotInCharge = 6,
        seqNumber = 7,
        startTime = 8,
        endTime = 9
    };

    bool updateTask(uint32_t id, TaskDao::Column column, std::string data);
};

#endif