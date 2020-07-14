#include "taskController.hpp"

TaskController::TaskController() 
{

}

TaskController::~TaskController(){}

void TaskController::setTaskList(std::vector<Task> &taskList)
{
    this->taskList.clear();
    std::copy(taskList.begin(), taskList.end(), std::back_inserter(this->taskList));
}

void TaskController::pop()
{
    if (taskList.size() != 0)
        taskList.erase(taskList.begin());
}

bool TaskController::deleteTaskById(uint32_t id)
{
    auto it = std::find_if(taskList.begin(), taskList.end(), [&id](const Task & t) -> bool { return t.id == id; });
    bool tst = it != taskList.end();
    if (tst)
        taskList.erase(it);

    return true;
}

Task *TaskController::getTaskById(uint32_t id)
{
    auto it = std::find_if(taskList.begin(), taskList.end(), [&id](const Task & t) -> bool { return t.id == id; });
    if (it != taskList.end())
        return &(*it);

    return NULL;
}

Task *TaskController::getTaskByIndex(uint32_t index)
{
    return &taskList[index];
}

std::vector<Task> *TaskController::getTaskList()
{
    return &taskList;
}