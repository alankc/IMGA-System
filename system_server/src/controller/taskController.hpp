#ifndef TASK_CONTROLLER_HPP
#define TASK_CONTROLLER_HPP

#include <vector>

#include "../model/task.hpp"
#include "../dao/taskDao.hpp"

class TaskController
{
private:
    TaskDao td;
    std::vector<Task> tasksToSchedule;

public:
    TaskController(GeneralDao *gDao);
    ~TaskController();

    std::vector<Task> &getTasksToSchedule();

    void updateTasksToSchedule();
    bool updateTaskScheduled(std::vector<uint32_t> tasks, std::vector<uint32_t> robots);
};

#endif