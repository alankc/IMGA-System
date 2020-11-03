#ifndef CHROMOSSOME_HPP
#define CHROMOSSOME_HPP

#include <vector>
#include <map>
#include <system_server/MsgTaskList.h>
#include <cstdint>
#include "../model/task.hpp"
#include "../model/robot.hpp"

struct ChromosomeResult
{
	uint32_t robotId;
	uint32_t taskId;
	bool scheduled;
	ChromosomeResult(uint32_t robotId, uint32_t taskId, bool scheduled) : robotId(robotId), taskId(taskId), scheduled(scheduled) {}
};

class Chromosome
{
private:
	static std::vector<Task> *taskList;
	static std::vector<Robot> *robotList;
	static std::vector<std::vector<double>> *distanceMaxtrix;
	static double startTime;

	std::vector<uint16_t> tasks;
	std::vector<uint16_t> robots;
	std::vector<bool> scheduled;
	double fitness;

public:
	Chromosome(bool initialize = false);
	~Chromosome();

	static void setTaskList(std::vector<Task> *taskList);
	static void setRobotList(std::vector<Robot> *robotList);
	static void setDistanceMatrix(std::vector<std::vector<double>> *distanceMaxtrix);
	static void setStartTime(double startTime);

	static Chromosome crossover(Chromosome &c1, Chromosome &c2);
	static Chromosome orderCrossover(Chromosome &c1, Chromosome &c2);
	static Chromosome orderBasedCrossover(Chromosome &c1, Chromosome &c2);
	static Chromosome positionBasedCrossover(Chromosome &c1, Chromosome &c2);

	static void mutate(Chromosome &c);
	static void exchangeMutation(Chromosome &c);
	static void displacementMutation(Chromosome &c);
	static void inversionMutation(Chromosome &c);
	static void scrambleMutation(Chromosome &c);

	void initialize();
	void computeFitness();
	std::vector<uint16_t> &getTasks();
	std::vector<uint16_t> &getRobots();
	std::vector<bool> &getScheduled();
	double getFitness() const;
	std::vector<ChromosomeResult> getResult();
	bool allScheduled();
	void printResult();
	void getResult(std::vector<uint32_t> &tasksScheduled, std::vector<uint32_t> &robots, std::vector<uint32_t> &tasksFailed);
	//std::vector<std::pair<robot's id, task list>> &listOfTaskList
	// std::vector<task's id> &taskFailedid
	void getResult(std::map<uint32_t, system_server::MsgTaskList> &listOfTaskList, std::vector<uint32_t> &taskFailedId);

	//Methods to help with statics
	double totalEnergy(bool allTasks);
	std::vector<double> totalEnergyByRobot(bool normalized, bool allTasks);
	std::vector<double> totalEnergyByTask(bool allTasks);
	double totalTime(bool allTasks);
	std::vector<double> totalTimeByRobot(bool allTasks);
	std::vector<double> totalTimeByTask(bool allTasks);
	uint64_t totalPayload(bool allTasks);
	std::vector<double> totalPayloadByRobot(bool normalized, bool allTasks);
	uint16_t numberOfScheduled();

	bool operator<(const Chromosome &chromosomeObj) const
	{
		return (this->fitness < chromosomeObj.fitness);
	}
};

#endif