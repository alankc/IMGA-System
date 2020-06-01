#include "taskController.hpp"

TaskController::TaskController(GeneralDao *gDao)
{
    this->td = TaskDao(gDao);
}

TaskController::~TaskController() {}

std::vector<Task> &TaskController::getTasksToSchedule()
{
    return tasksToSchedule;
}

void TaskController::updateTasksToSchedule()
{
    
}

bool TaskController::updateTaskScheduled(std::vector<uint32_t> tasks, std::vector<uint32_t> robots)
{
}