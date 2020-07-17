#include "taskController.hpp"

TaskController::TaskController()
{
    mtx = new std::mutex();
}

TaskController::~TaskController() {}

void TaskController::clear()
{
    mtx->lock();
    taskList.clear();
    mtx->unlock();
}

void TaskController::push(system_client::MsgTask &t)
{
    mtx->lock();
    taskList.push_back(t);
    mtx->unlock();
}

void TaskController::pop()
{
    mtx->lock();
    if (taskList.size() != 0)
        taskList.erase(taskList.begin());
    mtx->unlock();
}

bool TaskController::deleteTaskById(uint32_t id)
{
    mtx->lock();
    auto it = std::find_if(taskList.begin(), taskList.end(), [&id](const system_client::MsgTask &t) -> bool { return t.id == id; });
    bool tst = it != taskList.end();
    if (tst)
        taskList.erase(it);
    mtx->unlock();
    return tst;
}

bool TaskController::getFirst(system_client::MsgTask &t)
{
    mtx->lock();
    if (taskList.empty())
    {
        mtx->unlock();
        return false;
    }

    t = taskList[0];
    mtx->unlock();
    return true;
}

system_client::MsgTask *TaskController::getTaskById(uint32_t id)
{

    system_client::MsgTask *t = NULL;
    mtx->lock();
    auto it = std::find_if(taskList.begin(), taskList.end(), [&id](const system_client::MsgTask &t) -> bool { return t.id == id; });
    if (it != taskList.end())
        t = &(*it);
    mtx->unlock();
    return t;
}

system_client::MsgTask *TaskController::getTaskByIndex(uint32_t index)
{
    return &taskList[index];
}

std::vector<system_client::MsgTask> *TaskController::getTaskList()
{
    return &taskList;
}