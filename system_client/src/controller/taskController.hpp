#ifndef TASK_CONTROLLER_HPP
#define TASK_CONTROLLER_HPP

#include <vector>
#include <algorithm>
#include <system_client/MsgTask.h>

using namespace system_client;
typedef MsgTask Task;

class TaskController
{
private:
    std::vector<Task> taskList;

public:
    TaskController();
    ~TaskController();

    void setTaskList(std::vector<Task>& taskList);
    void pop();
    bool deleteTaskById(uint32_t id);
    Task *getTaskById(uint32_t id);
    Task *getTaskByIndex(uint32_t index);
    std::vector<Task> *getTaskList();
    
};

#endif