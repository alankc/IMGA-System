#ifndef TASK_CONTROLLER_HPP
#define TASK_CONTROLLER_HPP

#include <vector>

#include "../model/task.hpp"
#include "../model/robot.hpp"
#include "../model/location.hpp"
#include "../dao/taskDao.hpp"

#include "../scheduler/chromosome.hpp"

class TaskController
{
private:
    TaskDao td;
    std::vector<Task> tasksToSchedule;
    std::vector<Task> tasksRunning;

public:
    TaskController();
    TaskController(GeneralDao *gDao);
    ~TaskController();

    void updateTasksToSchedule(uint32_t numberOfTasks);
    bool updateTaskScheduled(std::vector<uint32_t> &tasksScheduled, std::vector<uint32_t> &robots, std::vector<uint32_t> &tasksFailed);
    //most of the paramters of system_server::MsgTask are no used in this method
    //but it make the callScheduler in generalController simple
    bool updateTaskScheduled(std::map<uint32_t, system_server::MsgTaskList> &listOfTaskList, std::vector<uint32_t> &taskFailedId);
    
    //updata tasks withour remove, when neessary...
    bool updateTask(uint32_t id, TaskDao::Column column, std::string data);
    //updatas task status. time is used when status are pickup ore sucess
    bool updateTaskStatus(uint32_t id, std::string status, double time = 0);
    bool getRobotincharge(uint32_t idTask, uint32_t &idRobot);
    
    //Will set all tasks with robot in charge = idRobot to failled and 
    //add that tasks to idTasksFromRobot
    bool setFail(uint32_t idRobot, std::vector<uint32_t> &idTasksFromRobot, double time);

    //Will check if there is tasks with missing deadline
    void deadlineCheck();
    //Will check if there is tasks to cancel requested by user
    void toCancelCheck();

    //Old functions...
    std::vector<Task> *getTasksToSchedule();
    Task *getTaskToScheduleById(uint32_t id);
    Task *getTaskToScheduleByIndex(uint32_t index);
    void copyTaskToSchedule(std::vector<Task> &copy);
    std::size_t getTaskToScheduleSize();

    Chromosome generateTasks(uint32_t numberOfTasks, std::vector<Robot> *freeRobotList, std::vector<std::vector<double>> *distanceMatrix, uint32_t numberOfDepots);
    Chromosome generateTasksCoordination(std::vector<Robot> *freeRobotList, std::vector<std::vector<double>> *distanceMatrix);
};

#endif