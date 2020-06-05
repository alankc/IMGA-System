#include "taskController.hpp"

TaskController::TaskController() {}

TaskController::TaskController(GeneralDao *gDao)
{
    this->td = TaskDao(gDao);
}

TaskController::~TaskController() {}

void TaskController::updateTasksToSchedule()
{
    tasksToSchedule.clear();
    bool tst = td.getTaskList(tasksToSchedule, Task::STATUS_NEW);
    if (!tst)
        std::cout << "Fail to update tasks to schedule" << std::endl;
    else
        std::cout << "Tasks to schedule has been updated" << std::endl;
}

bool TaskController::updateTaskScheduled(std::vector<uint32_t> tasks, std::vector<uint32_t> robots)
{
}

Task *TaskController::getTaskById(uint32_t id)
{
    auto it = std::find(tasksToSchedule.begin(), tasksToSchedule.end(), id);

    if (it != tasksToSchedule.end())
        return &(*it);

    return NULL;
}

Task *TaskController::getTaskByIndex(uint32_t index)
{
    return &tasksToSchedule[index];
}

void TaskController::copyTaskList(std::vector<Task> &copy)
{
    copy = std::vector<Task>(tasksToSchedule);
}

std::size_t TaskController::getTaskListSize()
{
    return tasksToSchedule.size();
}
