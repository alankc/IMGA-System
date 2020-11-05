#include "taskController.hpp"
#include <random>
#include <string>

TaskController::TaskController() {}

TaskController::TaskController(GeneralDao *gDao)
{
    this->td = TaskDao(gDao);
}

TaskController::~TaskController() {}

void TaskController::updateTasksToSchedule(uint32_t numberOfTasks)
{
    tasksToSchedule.clear();
    bool tst = td.getTaskList(tasksToSchedule, Task::STATUS_NEW, numberOfTasks);
    if (!tst)
        std::cout << "Fail to update tasks to schedule" << std::endl;
    else
        std::cout << "Tasks to schedule has been updated" << std::endl;
}

bool TaskController::updateTaskScheduled(std::vector<uint32_t> &tasksScheduled, std::vector<uint32_t> &robots, std::vector<uint32_t> &tasksFailed)
{
    std::cout << "!!! NOT IMPLEMENTED !!!\n";
    std::cout << "TaskController::updateTaskScheduled(std::vector<uint32_t> &tasksScheduled, std::vector<uint32_t> &robots, std::vector<uint32_t> &tasksFailed)\n";
    std::cout << "!!! NOT IMPLEMENTED !!!" << std::endl;
    return false;
}

bool TaskController::updateTaskScheduled(std::map<uint32_t, system_server::MsgTaskList> &listOfTaskList, std::vector<uint32_t> &taskFailedId)
{
    std::vector<TaskScheduledData> tsdList;

    //List of tasks scheduled to uptate
    for (auto ltl : listOfTaskList)
    {
        int64_t maxSeq;
        if (td.getMaximumSeqNum(ltl.first, maxSeq))
        {
            for (auto t : ltl.second.taskList)
            {
                TaskScheduledData tsd;
                tsd.id = t.id;
                tsd.seqNumber = ++maxSeq;
                tsd.status = Task::STATUS_SCHEDULED;
                tsd.robotInCharge = ltl.first;
                tsdList.push_back(tsd);

                //updating tasks running
                auto tasksToScheduleIt = std::find(tasksToSchedule.begin(), tasksToSchedule.end(), t.id);
                if (tasksToScheduleIt != tasksToSchedule.end())
                {
                    tasksToScheduleIt->setSeqNumber(tsd.seqNumber);
                    tasksToScheduleIt->setStatus(tsd.status);
                    tasksToScheduleIt->setRobotInCharge(tsd.robotInCharge);
                    tasksRunning.push_back(*tasksToScheduleIt);
                }
            }
        }
        else
            return false;
    }

    //List of tasks failed to uptate
    for (auto tiD : taskFailedId)
    {
        TaskScheduledData tsd;
        tsd.id = tiD;
        tsd.status = Task::STATUS_FAILED;
        tsdList.push_back(tsd);
    }

    return td.updateTasksScheduled(tsdList);
}

bool TaskController::updateTask(uint32_t id, TaskDao::Column column, std::string data)
{
    auto it = std::find(tasksRunning.begin(), tasksRunning.end(), id);
    if (it != tasksRunning.end())
    {
        switch (column)
        {
        case (TaskDao::Column::status):
            it->setStatus(data);
            break;

        case (TaskDao::Column::seqNumber):
            it->setSeqNumber(std::stoul(data, nullptr, 0));
            break;

        case (TaskDao::Column::startTime):
            it->setStartTime(std::stod(data));
            break;

        case (TaskDao::Column::endTime):
            it->setEndTime(std::stod(data));
            break;

        case (TaskDao::Column::robotInCharge):
            it->setRobotInCharge(std::stoul(data, nullptr, 0));
            break;

        case (TaskDao::Column::description):
            it->setDescription(data);
            break;

        case (TaskDao::Column::pickUpLocation):
            it->setPickUpLocation(std::stoul(data, nullptr, 0));
            break;

        case (TaskDao::Column::deliveryLocation):
            it->setDeliveryLocation(std::stoul(data, nullptr, 0));
            break;

        case (TaskDao::Column::payload):
            it->setPayload(std::stoul(data, nullptr, 0));
            break;

        case (TaskDao::Column::deadline):
            it->setDeadline(std::stod(data));
            break;

        default:
            return false;
        }
        td.updateTask(id, column, data);
        return true;
    }
    return false;
}

bool TaskController::updateTaskStatus(uint32_t id, std::string status, double time)
{
    auto it = std::find(tasksRunning.begin(), tasksRunning.end(), id);
    if (it != tasksRunning.end())
    {
        if (status == Task::STATUS_CANCELLED)
        {
            if (it->getStatus() == Task::STATUS_TO_CANCEL_USER)
                it->setStatus(Task::STATUS_CANCELLED_USER);
            else if (it->getStatus() == Task::STATUS_TO_CANCEL_DEADLINE)
                it->setStatus(Task::STATUS_CANCELLED_DEADLINE);
            else
                return false;
        }
        else
            it->setStatus(status);

        td.updateTask(id, TaskDao::status, it->getStatus());

        if (status == Task::STATUS_PERFORMING_PICK_UP)
        {
            it->setStartTime(time);
            td.updateTask(id, TaskDao::startTime, std::to_string(time));
        }
        else if ((status == Task::STATUS_SUCESS) ||
                 (status == Task::STATUS_FAILED) ||
                 (status == Task::STATUS_CANCELLED))
        {
            it->setEndTime(time);
            td.updateTask(id, TaskDao::endTime, std::to_string(time));
            tasksRunning.erase(it);
        }

        return true;
    }
    return false;
}

bool TaskController::getRobotincharge(uint32_t idTask, uint32_t &idRobot)
{
    auto it = std::find(tasksRunning.begin(), tasksRunning.end(), idTask);

    if (it != tasksRunning.end())
    {
        idRobot = it->getRobotInCharge();
        return true;
    }

    return false;
}

bool TaskController::setFail(uint32_t idRobot, std::vector<uint32_t> &idTasksFromRobot, double time)
{
    for (uint32_t i = 0; i < tasksRunning.size(); i++)
    {
        Task &t = tasksRunning[i];
        if (t.getRobotInCharge() == idRobot)
        {
            idTasksFromRobot.push_back(t.getId());
            t.setStatus(Task::STATUS_FAILED);
            t.setEndTime(time);
            td.updateTask(t.getId(), TaskDao::endTime, std::to_string(time));
            td.updateTask(t.getId(), TaskDao::status, Task::STATUS_FAILED);
            tasksRunning.erase(tasksRunning.begin() + i);
        }
    }
}

void TaskController::deadlineCheck(std::vector<uint32_t> &idTasksToCancel, double time)
{
    for (auto &t : tasksRunning)
    {
        if (time >= t.getDeadline())
        {
            auto stt = t.getStatus();
            //If not started yet or just going to pickup it can be canceled
            if ((stt == Task::STATUS_SCHEDULED) || (stt == Task::STATUS_PERFORMING_PICK_UP))
            {
                t.setStatus(Task::STATUS_TO_CANCEL_DEADLINE);
                idTasksToCancel.push_back(t.getId());
            }
        }
    }
}

void TaskController::toCancelCheck(std::vector<uint32_t> &idTasksToCancel)
{
    td.getTaskIdList(idTasksToCancel, Task::STATUS_TO_CANCEL_USER, 100);
    for (auto &t : idTasksToCancel)
    {
        auto it = std::find(tasksRunning.begin(), tasksRunning.end(), t);
        if (it != tasksRunning.end())
            it->setStatus(Task::STATUS_TO_CANCEL_USER);
    }
}

std::vector<Task> *TaskController::getTasksToSchedule()
{
    return &tasksToSchedule;
}

Task *TaskController::getTaskToScheduleById(uint32_t id)
{
    auto it = std::find(tasksToSchedule.begin(), tasksToSchedule.end(), id);

    if (it != tasksToSchedule.end())
        return &(*it);

    return NULL;
}

Task *TaskController::getTaskToScheduleByIndex(uint32_t index)
{
    return &tasksToSchedule[index];
}

void TaskController::copyTaskToSchedule(std::vector<Task> &copy)
{
    copy = std::vector<Task>(tasksToSchedule);
}

std::size_t TaskController::getTaskToScheduleSize()
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
    tasksToSchedule.clear();
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
    Chromosome::setTaskList(&tasksToSchedule);
    Chromosome::setRobotList(freeRobotList);
    Chromosome::setDistanceMatrix(distanceMatrix);
    auto &cs = rst.getScheduled();
    for (uint32_t i = 0; i < numberOfTasks; i++)
        cs.push_back(false);
    auto &ct = rst.getTasks();
    std::copy(indxVec.begin(), indxVec.end(), std::back_inserter(ct));
    auto &cr = rst.getRobots();

    for (uint32_t i = 0; i < numberOfTasks; i++)
    {
        Task &t = tasksToSchedule[indxVec[i]];
        uint32_t robotIndex = t.getRobotInCharge();
        Robot &r = robotList[robotIndex];

        cr.push_back(robotIndex);

        double timeToPickUp = r.computeTimeRequirement(dm[r.getCurrentLocation()][t.getPickUpLocation()]);
        double timeToDelivery = r.computeTimeRequirement(dm[t.getPickUpLocation()][t.getDeliveryLocation()]);
        //add 10% de folga
        double timeInTravel = (timeToPickUp + timeToDelivery) * (1.0 + 10.0 / 100.0);

        double battery = r.computeBatteryRequirement(timeInTravel) + freeRobotList->at(robotIndex).getRemainingBattery();
        freeRobotList->at(robotIndex).setRemainingBattery(battery);
        r.setCurrentLocation(t.getDeliveryLocation());

        timeDemand[robotIndex] += timeInTravel;
        t.setDeadline(timeDemand[robotIndex]);
    }

    rst.computeFitness();

    return rst;
}

Chromosome TaskController::generateTasksCoordination(std::vector<Robot> *freeRobotList, std::vector<std::vector<double>> *distanceMatrix)
{
    auto robotList = *freeRobotList;
    auto &dm = *distanceMatrix;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint32_t> clusterSelectDist(0, 11);
    std::vector<std::uniform_int_distribution<uint32_t>> clusterVectorDist;
    uint32_t bias = 13;
    uint32_t clusterSize = 16;
    for (uint32_t i = 0; i < 12; i++)
    {
        std::uniform_int_distribution<uint32_t> tmpDist(bias + i * clusterSize, bias + (i + 1) * clusterSize - 1);
        clusterVectorDist.push_back(tmpDist);
    }

    Chromosome cResult;
    Chromosome::setDistanceMatrix(distanceMatrix);
    Chromosome::setRobotList(freeRobotList);
    Chromosome::setTaskList(&tasksToSchedule);

    auto &chrTasks = cResult.getTasks();
    chrTasks.clear();
    auto &chrRobots = cResult.getRobots();
    chrRobots.clear();
    auto &chrScheduled = cResult.getScheduled();

    uint32_t taskId = 0;
    tasksToSchedule.clear();
    for (uint32_t i = 0; i < robotList.size(); i++)
    {
        auto &r = robotList[i];
        double totalTime = 0;
        if (i == 0) //If robot 0, 4 tasks
        {
            for (uint32_t j = 0; j < 4; j++)
            {
                Task t;
                t.setId(taskId);
                t.setDescription("Task " + std::to_string(taskId));
                t.setStatus(Task::STATUS_NEW);
                t.setPayload(robotList[i].getMaximumPayload());

                uint32_t clusterP = clusterSelectDist(gen);
                uint32_t pickUp = clusterVectorDist[clusterP](gen);
                t.setPickUpLocation(pickUp);
                t.setDeliveryLocation(i + 8); //Each robot has an delivert location (task depot)

                double timeToPickUp = r.computeTimeRequirement(dm[r.getCurrentLocation()][t.getPickUpLocation()]);
                double timeToDelivery = r.computeTimeRequirement(dm[t.getPickUpLocation()][t.getDeliveryLocation()]);
                //add 10% de folga
                double timeInTravel = (timeToPickUp + timeToDelivery + 2.0 * r.getWaitingTime()) * (1.0 + 10.0 / 100.0);
                totalTime += std::ceil(timeInTravel);
                t.setDeadline(totalTime);

                t.setRobotInCharge(i);
                tasksToSchedule.push_back(t);

                chrRobots.push_back(i);
                chrTasks.push_back(taskId);

                r.setCurrentLocation(t.getDeliveryLocation());

                taskId++;
            }
        }
        else if (i == robotList.size() - 1) //if robot 5, 2 tasks
        {
            for (uint32_t j = 0; j < 2; j++)
            {
                Task t;
                t.setId(taskId);
                t.setDescription("Task " + std::to_string(taskId));
                t.setStatus(Task::STATUS_NEW);
                t.setPayload(robotList[i].getMaximumPayload());

                uint32_t clusterP = clusterSelectDist(gen);
                uint32_t pickUp = clusterVectorDist[clusterP](gen);
                t.setPickUpLocation(pickUp);
                t.setDeliveryLocation(i + 8); //Each robot has an delivert location (task depot)

                double timeToPickUp = r.computeTimeRequirement(dm[r.getCurrentLocation()][t.getPickUpLocation()]);
                double timeToDelivery = r.computeTimeRequirement(dm[t.getPickUpLocation()][t.getDeliveryLocation()]);
                //add 10% de folga
                double timeInTravel = (timeToPickUp + timeToDelivery + 2.0 * r.getWaitingTime()) * (1.0 + 10.0 / 100.0);
                totalTime += std::ceil(timeInTravel);
                t.setDeadline(totalTime);

                t.setRobotInCharge(i);
                tasksToSchedule.push_back(t);

                chrRobots.push_back(i);
                chrTasks.push_back(taskId);

                r.setCurrentLocation(t.getDeliveryLocation());

                taskId++;
            }
        }
        else //others robots 3 tasks
        {
            for (uint32_t j = 0; j < 3; j++)
            {
                Task t;
                t.setId(taskId);
                t.setDescription("Task " + std::to_string(taskId));
                t.setStatus(Task::STATUS_NEW);
                t.setPayload(robotList[i].getMaximumPayload());

                uint32_t clusterP = clusterSelectDist(gen);
                uint32_t pickUp = clusterVectorDist[clusterP](gen);
                t.setPickUpLocation(pickUp);
                t.setDeliveryLocation(i + 8); //Each robot has an delivert location (task depot)

                double timeToPickUp = r.computeTimeRequirement(dm[r.getCurrentLocation()][t.getPickUpLocation()]);
                double timeToDelivery = r.computeTimeRequirement(dm[t.getPickUpLocation()][t.getDeliveryLocation()]);
                //add 10% de folga
                double timeInTravel = (timeToPickUp + timeToDelivery + 2.0 * r.getWaitingTime()) * (1.0 + 10.0 / 100.0);
                totalTime += std::ceil(timeInTravel);
                t.setDeadline(totalTime);

                t.setRobotInCharge(i);
                tasksToSchedule.push_back(t);

                chrRobots.push_back(i);
                chrTasks.push_back(taskId);

                r.setCurrentLocation(t.getDeliveryLocation());

                taskId++;
            }
        }
    }
    chrScheduled = std::vector<bool>(tasksToSchedule.size(), true);
    cResult.computeFitness();
    return cResult;
}