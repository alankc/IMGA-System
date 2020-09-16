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

Chromosome TaskController::generateTasks(uint32_t numberOfTasks, std::vector<Robot> *freeRobotList, std::vector<std::vector<double>> *distanceMatrix, uint32_t numberOfDepots)
{
    auto robotList = *freeRobotList;
    auto &dm = *distanceMatrix;

    std::random_device rd;
    std::mt19937 gen(rd());
    //std::uniform_int_distribution<uint32_t> depotDist(0, numberOfDepots - 1);
    std::uniform_int_distribution<uint32_t> locationDist(numberOfDepots, dm.size() - 1);
    std::uniform_int_distribution<uint32_t> robotDist(0, robotList.size() - 1);

    uint32_t minPayload = robotList[0].getMaximumPayload();
    for (auto r : robotList)
    {
        uint32_t tmpPayload = r.getMaximumPayload();
        if (minPayload > tmpPayload)
            minPayload = tmpPayload;
    }

    //Defining vector of tasks
    for (uint32_t i = 0; i < numberOfTasks; i++)
    {
        Task t;
        uint32_t robotIndex = robotDist(gen);
        Robot &r = robotList[robotIndex];

        t.setId(i);
        t.setDescription("Task " + std::to_string(i));
        t.setStatus(Task::STATUS_NEW);
        t.setPickUpLocation(locationDist(gen));
        t.setDeliveryLocation(locationDist(gen));
        while (t.getPickUpLocation() == t.getDeliveryLocation())
            t.setDeliveryLocation(locationDist(gen));
        t.setRobotInCharge(robotIndex);
        std::uniform_int_distribution<uint32_t> payloadDist(minPayload, r.getMaximumPayload());
        t.setPayload(payloadDist(gen));

        tasksToSchedule.push_back(t);
    }

    //defining sequence of execution
    //it is need to generate goal cromossome with task sequecen shuffled
    std::vector<double> timeDemand(robotList.size(), 0.0);

    std::vector<uint32_t> indxVec(numberOfTasks);
    std::iota(std::begin(indxVec), std::end(indxVec), 0);
    std::shuffle(std::begin(indxVec), std::end(indxVec), gen);


    Chromosome rst;
    rst.setTaskList(&tasksToSchedule);
    rst.setRobotList(freeRobotList);
    rst.setDistanceMatrix(distanceMatrix);
    auto &cs = rst.getScheduled();
    cs = std::vector<bool>(tasksToSchedule.size(), false);
    auto &ct = rst.getTasks();
    std::copy(indxVec.begin(), indxVec.end(), std::back_inserter(ct));
    auto &cr = rst.getRobots();

    for (uint32_t i = 0; i < numberOfTasks; i++)
    {
        Task& t = tasksToSchedule[indxVec[i]];
        uint32_t robotIndex = t.getRobotInCharge();
        Robot &r = robotList[robotIndex];

        cr.push_back(robotIndex);

        double timeToPickUp = r.computeTimeRequirement(dm[r.getCurrentLocation()][t.getPickUpLocation()]);
        double timeToDelivery = r.computeTimeRequirement(dm[t.getPickUpLocation()][t.getDeliveryLocation()]);
        //add 5% de folga
        double timeInTravel = (timeToPickUp + timeToDelivery) * (1.0 + 5.0 / 100.0);

        double battery = r.computeBatteryRequirement(timeInTravel) + freeRobotList->at(robotIndex).getRemainingBattery();
        freeRobotList->at(robotIndex).setRemainingBattery(battery);
        r.setCurrentLocation(t.getDeliveryLocation());

        timeDemand[robotIndex] += timeInTravel;
        t.setDeadline(timeDemand[robotIndex]);
    }

    rst.computeFitness();

    return rst;
}