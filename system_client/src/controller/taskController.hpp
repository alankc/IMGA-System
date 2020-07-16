#ifndef TASK_CONTROLLER_HPP
#define TASK_CONTROLLER_HPP

#include <vector>
#include <algorithm>
#include <system_client/MsgTask.h>

class TaskController
{
private:
    std::vector<system_client::MsgTask> taskList;

public:
    TaskController();
    ~TaskController();

    void clear();
    void push(system_client::MsgTask& t);
    void pop();
    bool deleteTaskById(uint32_t id);
    bool getFirst(system_client::MsgTask& t);
    system_client::MsgTask *getTaskById(uint32_t id);
    system_client::MsgTask *getTaskByIndex(uint32_t index);
    std::vector<system_client::MsgTask> *getTaskList();
    
};

#endif