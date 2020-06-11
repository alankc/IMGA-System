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
    TaskController();
    TaskController(GeneralDao *gDao);
    ~TaskController();

    void updateTasksToSchedule();
    bool updateTaskScheduled(std::vector<uint32_t> tasks, std::vector<uint32_t> robots);

    std::vector<Task> *getTaskList();
    Task *getTaskById(uint32_t id);
    Task *getTaskByIndex(uint32_t index);
    void copyTaskList(std::vector<Task> &copy);
    std::size_t getTaskListSize();
};

#endif