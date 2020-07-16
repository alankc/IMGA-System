#include "taskController.hpp"

TaskController::TaskController() 
{

}

TaskController::~TaskController(){}

void TaskController::clear()
{
    taskList.clear();
}

void TaskController::push(system_client::MsgTask& t)
{
    taskList.push_back(t);
}

void TaskController::pop()
{
    if (taskList.size() != 0)
        taskList.erase(taskList.begin());
}

bool TaskController::deleteTaskById(uint32_t id)
{
    auto it = std::find_if(taskList.begin(), taskList.end(), [&id](const system_client::MsgTask & t) -> bool { return t.id == id; });
    bool tst = it != taskList.end();
    if (tst)
        taskList.erase(it);

    return true;
}

bool TaskController::getFirst(system_client::MsgTask& t)
{
    if (taskList.empty())
        return false;

    t = taskList[0];
    return true;    
}

system_client::MsgTask *TaskController::getTaskById(uint32_t id)
{
    auto it = std::find_if(taskList.begin(), taskList.end(), [&id](const system_client::MsgTask & t) -> bool { return t.id == id; });
    if (it != taskList.end())
        return &(*it);

    return NULL;
}

system_client::MsgTask *TaskController::getTaskByIndex(uint32_t index)
{
    return &taskList[index];
}

std::vector<system_client::MsgTask> *TaskController::getTaskList()
{
    return &taskList;
}