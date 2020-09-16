#include "taskController.hpp"
#include <random>

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

bool TaskController::updateTaskScheduled(std::vector<uint32_t> &tasksScheduled, std::vector<uint32_t> &robots, std::vector<uint32_t> &tasksFailed)
{
}

std::vector<Task> *TaskController::getTaskList()
{
    return &tasksToSchedule;
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

Chromosome TaskController::generateTasks(uint32_t numberOfTasks, std::vector<Robot> *freeRobotList, std::vector<std::vector<double>> *distanceMatrix, uint32_t numberOfPlaces, uint32_t numberOfDepots)
{
    auto &robotList = *freeRobotList;
    auto &dm = *distanceMatrix;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint32_t> depotDist(0, numberOfDepots - 1);
    std::uniform_int_distribution<uint32_t> placeDist(numberOfDepots, numberOfPlaces - 1);
    std::uniform_int_distribution<uint32_t> robotDist(0, robotList.size() - 1);

    std::vector<double> timeDemand(robotList.size(), 0.0);

    Chromosome result;

    for (uint32_t i = 0; i < numberOfTasks; i++)
    {
        Task t;
        t.setId(i)
    }
}