#include "chromosome.hpp"

#include <iostream>
#include <iomanip>
#include <random>
#include <algorithm>
#include <set>

std::vector<Task> *Chromosome::taskList = NULL;
std::vector<Robot> *Chromosome::robotList = NULL;
std::vector<std::vector<double>> *Chromosome::distanceMaxtrix = NULL;

Chromosome::Chromosome(bool initialize)
{
	if (initialize)
		this->initialize();
}

Chromosome::~Chromosome()
{
}

void Chromosome::setTaskList(std::vector<Task> *taskList)
{
	Chromosome::taskList = taskList;
}

void Chromosome::setRobotList(std::vector<Robot> *robotList)
{
	Chromosome::robotList = robotList;
}

void Chromosome::setDistanceMatrix(std::vector<std::vector<double>> *distanceMaxtrix)
{
	Chromosome::distanceMaxtrix = distanceMaxtrix;
}

Chromosome Chromosome::crossover(Chromosome &c1, Chromosome &c2)
{
	static std::random_device rd;  //Will be used to obtain a seed for the random number engine
	static std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
	static std::uniform_int_distribution<uint8_t> dist(0, 1);

	Chromosome *ind[2];
	ind[0] = &c1;
	ind[1] = &c2;

	Chromosome result;
	result.getScheduled().resize(taskList->size(), false);

	std::vector<uint32_t> idCrossover;
	std::set<uint16_t> posCrossover;

	std::copy(c2.getTasks().begin(), c2.getTasks().end(), std::back_inserter(result.getTasks()));

	for (uint16_t i = 0; i < taskList->size(); i++)
	{
		result.getRobots().push_back(ind[dist(gen)]->getRobots()[i]);

		if (dist(gen))
		{
			idCrossover.push_back(c1.getTasks()[i]);
			auto it = std::find(c2.getTasks().begin(), c2.getTasks().end(), c1.getTasks()[i]);
			posCrossover.insert(std::distance(c2.getTasks().begin(), it));
		}
	}

	uint16_t i = 0;
	for (auto it : posCrossover)
	{
		result.getTasks()[it] = idCrossover[i++];
	}

	result.computeFitness();

	return result;
}

Chromosome Chromosome::orderCrossover(Chromosome &c1, Chromosome &c2)
{
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::uniform_int_distribution<uint8_t> dist(0, 1);
	std::uniform_int_distribution<uint16_t> sizeDist(0, taskList->size() - 1);

	Chromosome result;
	result.getScheduled().resize(taskList->size(), false);

	//Just improving diversity
	Chromosome *cr1, *cr2;
	if (dist(gen))
	{
		cr1 = &c1;
		cr2 = &c2;
	}
	else
	{
		cr1 = &c2;
		cr2 = &c1;
	}

	//Computing begin and end cut
	uint16_t begin = sizeDist(gen);
	uint16_t end = sizeDist(gen);
	while ((begin == end) || (end - begin - taskList->size() + 1 == 0))
	{
		begin = sizeDist(gen);
		end = sizeDist(gen);
	}
	if (begin > end)
		std::swap(begin, end);

	//preparing task part of chromosome
	std::vector<uint16_t> midleVec;
	std::copy(cr1->getTasks().begin() + begin, cr1->getTasks().begin() + end + 1, std::back_inserter(midleVec));
	result.getTasks().resize(taskList->size());

	uint16_t i = (end + 1) % taskList->size(); //It is to travel through c2
	uint16_t j = (end + 1) % taskList->size(); //It is to travel through c1

	//preparing robot part of chromosome
	Chromosome *ind[2];
	ind[0] = cr1;
	ind[1] = cr2;

	for (uint16_t k = 0; k < taskList->size(); k++)
	{
		//robot
		result.getRobots().push_back(ind[dist(gen)]->getRobots()[k]);

		//tasks
		if ((j >= begin) && (j <= end))
		{
			result.getTasks()[j] = cr1->getTasks()[j];
		}
		else
		{
			while (true)
			{
				uint16_t seqNumber = cr2->getTasks()[i];
				i = (i + 1) % taskList->size();
				auto it = std::find(midleVec.begin(), midleVec.end(), seqNumber);
				if (it == midleVec.end())
				{
					result.getTasks()[j] = seqNumber;
					break;
				}
			}
		}
		j = (j + 1) % taskList->size();
	}
	result.computeFitness();
	return result;
}

Chromosome Chromosome::orderBasedCrossover(Chromosome &c1, Chromosome &c2)
{
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::uniform_int_distribution<uint8_t> dist(0, 1);

	Chromosome result;
	result.getScheduled().resize(taskList->size(), false);

	//Just improving diversity
	Chromosome *cr1, *cr2;
	if (dist(gen))
	{
		cr1 = &c1;
		cr2 = &c2;
	}
	else
	{
		cr1 = &c2;
		cr2 = &c1;
	}

	//preparing robot part of chromosome
	Chromosome *ind[2];
	ind[0] = cr1;
	ind[1] = cr2;

	//preparing task part of chromosome
	std::copy(cr2->getTasks().begin(), cr2->getTasks().end(), std::back_inserter(result.getTasks()));
	std::vector<uint16_t> valueVec;
	std::set<uint16_t> positionVec;

	for (uint16_t i = 0; i < taskList->size(); i++)
	{
		//Robots
		result.getRobots().push_back(ind[dist(gen)]->getRobots()[i]);

		//Tasks
		if (dist(gen))
		{
			uint16_t value = cr1->getTasks()[i];
			auto it = std::find(cr2->getTasks().begin(), cr2->getTasks().end(), value);
			uint16_t position = std::distance(cr2->getTasks().begin(), it);
			valueVec.push_back(value);
			positionVec.insert(position);
		}
	}

	//Tasks
	uint j = 0;
	for (uint16_t i : positionVec)
		result.getTasks()[i] = valueVec[j++];

	result.computeFitness();
	return result;
}

Chromosome Chromosome::positionBasedCrossover(Chromosome &c1, Chromosome &c2)
{
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::uniform_int_distribution<uint8_t> dist(0, 1);

	Chromosome result;
	result.getScheduled().resize(taskList->size(), false);

	//Just improving diversity
	Chromosome *cr1, *cr2;
	if (dist(gen))
	{
		cr1 = &c1;
		cr2 = &c2;
	}
	else
	{
		cr1 = &c2;
		cr2 = &c1;
	}

	//preparing robot part of chromosome
	Chromosome *ind[2];
	ind[0] = &c1;
	ind[1] = &c2;

	//preparing task part of chromosome
	std::vector<uint16_t> valueVec;
	std::vector<bool> positionVec(taskList->size(), false);

	for (uint16_t i = 0; i < taskList->size(); i++)
	{
		if (dist(gen))
		{
			positionVec[i] = true;
			valueVec.push_back(cr1->getTasks()[i]);
		}
	}

	uint16_t j = 0;
	for (uint16_t i = 0; i < taskList->size(); i++)
	{
		//Robots
		result.getRobots().push_back(ind[dist(gen)]->getRobots()[i]);

		//Tasks
		if (positionVec[i])
		{
			result.getTasks().push_back(cr1->getTasks()[i]);
		}
		else
		{
			while (true)
			{
				uint16_t value = cr2->getTasks()[j++];
				auto itCr2 = std::find(valueVec.begin(), valueVec.end(), value);
				if (itCr2 == valueVec.end())
				{
					result.getTasks().push_back(value);
					break;
				}
			}
		}
	}
	result.computeFitness();
	return result;
}

void Chromosome::mutate(Chromosome &c)
{
	static std::random_device rd;  //Will be used to obtain a seed for the random number engine
	static std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
	std::uniform_int_distribution<uint16_t> robotDist(0, robotList->size() - 1);
	std::uniform_int_distribution<uint16_t> sizeDist(0, taskList->size() - 1);
	static std::uniform_int_distribution<uint8_t> dist(0, 1);

	if (dist(gen))
	{
		for (uint16_t i = 0; i < c.getTasks().size() * 0.5; i++)
		{
			uint16_t first = sizeDist(gen);
			uint16_t second = sizeDist(gen);
			std::swap(c.getRobots()[first], c.getRobots()[second]);

			first = sizeDist(gen);
			second = sizeDist(gen);
			std::swap(c.getTasks()[first], c.getTasks()[second]);
		}
	}
	else
	{
		for (uint16_t i = 0; i < c.getTasks().size() * 0.5; i++)
		{
			c.getRobots()[sizeDist(gen)] = robotDist(gen);

			uint16_t first = sizeDist(gen);
			uint16_t second = sizeDist(gen);
			std::swap(c.getTasks()[first], c.getTasks()[second]);
		}
	}

	c.computeFitness();
}

void Chromosome::exchangeMutation(Chromosome &c)
{
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::uniform_int_distribution<uint8_t> dist(0, 1);
	std::uniform_int_distribution<uint16_t> robotDist(0, robotList->size() - 1);
	std::uniform_int_distribution<uint16_t> sizeDist(0, taskList->size() - 1);

	uint16_t first = sizeDist(gen);
	uint16_t second = sizeDist(gen);

	if (dist(gen))
		std::swap(c.getTasks()[first], c.getTasks()[second]);
	else
		std::swap(c.getRobots()[first], c.getRobots()[second]);

	if (dist(gen))
		c.getRobots()[sizeDist(gen)] = robotDist(gen);

	c.computeFitness();
}

void Chromosome::displacementMutation(Chromosome &c)
{
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::uniform_int_distribution<uint8_t> dist(0, 1);
	std::uniform_int_distribution<uint16_t> robotDist(0, robotList->size() - 1);
	std::uniform_int_distribution<uint16_t> sizeDist(0, taskList->size() - 1);

	//Computing begin and end cut
	uint16_t begin = sizeDist(gen);
	uint16_t end = sizeDist(gen);
	while ((begin == end) || (end - begin - taskList->size() + 1 == 0))
	{
		begin = sizeDist(gen);
		end = sizeDist(gen);
	}
	if (begin > end)
		std::swap(begin, end);

	//defining size of cutted vector and insertion point
	uint16_t newSize = taskList->size() - 1 - end + begin;
	std::uniform_int_distribution<uint16_t> insertionDist(0, newSize);
	uint16_t insertion = insertionDist(gen);

	std::vector<uint16_t> tempVec;
	std::vector<uint16_t> midleVec;
	std::vector<uint16_t> *originalVec;

	if (dist(gen))
		originalVec = &c.getTasks();
	else
		originalVec = &c.getRobots();

	//filling in temporary vectors
	uint16_t i;
	for (i = 0; i < begin; i++)
		tempVec.push_back((*originalVec)[i]);
	for (; i <= end; i++)
		midleVec.push_back((*originalVec)[i]);
	for (; i < taskList->size(); i++)
		tempVec.push_back((*originalVec)[i]);

	//filling in the result
	originalVec->clear();
	for (i = 0; i < insertion; i++)
		originalVec->push_back(tempVec[i]);
	for (uint16_t j = 0; j < midleVec.size(); j++)
		originalVec->push_back(midleVec[j]);
	for (; i < newSize; i++)
		originalVec->push_back(tempVec[i]);

	if (dist(gen))
		c.getRobots()[sizeDist(gen)] = robotDist(gen);

	c.computeFitness();
}

void Chromosome::inversionMutation(Chromosome &c)
{
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::uniform_int_distribution<uint8_t> dist(0, 1);
	std::uniform_int_distribution<uint16_t> robotDist(0, robotList->size() - 1);
	std::uniform_int_distribution<uint16_t> sizeDist(0, taskList->size() - 1);

	//Computing begin and end cut
	uint16_t begin = sizeDist(gen);
	uint16_t end = sizeDist(gen);
	while ((begin == end) || (end - begin - taskList->size() + 1 == 0))
	{
		begin = sizeDist(gen);
		end = sizeDist(gen);
	}
	if (begin > end)
		std::swap(begin, end);

	//defining size of cutted vector and insertion point
	uint16_t newSize = taskList->size() - 1 - end + begin;
	std::uniform_int_distribution<uint16_t> insertionDist(0, newSize);
	uint16_t insertion = insertionDist(gen);

	std::vector<uint16_t> tempVec;
	std::vector<uint16_t> midleVec;
	std::vector<uint16_t> *originalVec;

	if (dist(gen))
		originalVec = &c.getTasks();
	else
		originalVec = &c.getRobots();

	//filling in temporary vectors
	uint16_t i;
	for (i = 0; i < begin; i++)
		tempVec.push_back((*originalVec)[i]);
	for (; i <= end; i++)
		midleVec.push_back((*originalVec)[i]);
	for (; i < taskList->size(); i++)
		tempVec.push_back((*originalVec)[i]);

	//filling in the result
	originalVec->clear();
	for (i = 0; i < insertion; i++)
		originalVec->push_back(tempVec[i]);
	for (int32_t j = midleVec.size() - 1; j >= 0; j--)
		originalVec->push_back(midleVec[j]);
	for (; i < newSize; i++)
		originalVec->push_back(tempVec[i]);

	if (dist(gen))
		c.getRobots()[sizeDist(gen)] = robotDist(gen);

	c.computeFitness();
}

void Chromosome::scrambleMutation(Chromosome &c)
{
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::uniform_int_distribution<uint8_t> dist(0, 1);
	std::uniform_int_distribution<uint16_t> robotDist(0, robotList->size() - 1);
	std::uniform_int_distribution<uint16_t> sizeDist(0, taskList->size() - 1);

	//Computing begin and end
	uint16_t begin = sizeDist(gen);
	uint16_t end = sizeDist(gen);
	while ((begin == end) || (end - begin - taskList->size() + 1 == 0))
	{
		begin = sizeDist(gen);
		end = sizeDist(gen);
	}
	if (begin > end)
		std::swap(begin, end);

	//shuffle
	if (dist(gen))
		std::shuffle(c.getTasks().begin() + begin, c.getTasks().begin() + end + 1, gen);
	else
		std::shuffle(c.getRobots().begin() + begin, c.getRobots().begin() + end + 1, gen);

	if (dist(gen))
		c.getRobots()[sizeDist(gen)] = robotDist(gen);

	c.computeFitness();
}

void Chromosome::computeFitness()
{
	std::vector<Robot> robotsCopy;
	std::copy(robotList->begin(), robotList->end(), std::back_inserter(robotsCopy));
	std::vector<double> timeDemand(robotList->size(), 0.0);
	std::vector<double> batteryDemand(robotList->size(), 0.0);
	uint16_t deadlineAttended = 0;
	uint16_t batteryAttended = 0;
	uint16_t payloadAttended = 0;
	double totalBatteryScheduled = 0;
	double totalPayloadScheduled = 0;
	double totalPayload = 0;
	double totalBatteryInTask = 0;
	for (uint16_t i = 0; i < taskList->size(); i++)
	{
		auto task = taskList->at(tasks[i]);
		auto robot = robotsCopy[robots[i]];
		totalPayload += task.getPayload();

		//Computing time and battery requirements
		double distanceToTheTask = distanceMaxtrix->at(robot.getCurrentLocation()).at(task.getPickUpLocation());
		double distanceInTask = distanceMaxtrix->at(task.getPickUpLocation()).at(task.getDeliveryLocation());
		double totalDistance = distanceToTheTask + distanceInTask;
		double timeInTask = robot.computeTimeRequirement(distanceInTask);
		double totalTime = robot.computeTimeRequirement(totalDistance);
		totalBatteryInTask += robot.computeBatteryRequirement(timeInTask);
		double batteryToPerformTask = robot.computeBatteryRequirement(totalTime);
		double totalBattery = batteryToPerformTask + batteryDemand[robots[i]];
		totalTime += timeDemand[robots[i]];

		bool timeTst = totalTime <= task.getDeadline();
		bool batteryTst = totalBattery <= robot.getUtilRemainingBattery();
		bool payloadTst = task.getPayload() <= robot.getMaximumPayload();

		if (timeTst && batteryTst && payloadTst)
		{
			scheduled[tasks[i]] = true;

			//updating robot position
			robotsCopy[robots[i]].setCurrentLocation(task.getDeliveryLocation());

			totalBatteryScheduled += batteryToPerformTask;
			totalPayloadScheduled += task.getPayload();

			//updating time and battery requirements
			timeDemand[robots[i]] = totalTime;
			batteryDemand[robots[i]] = totalBattery;

			deadlineAttended++;
			batteryAttended++;
			payloadAttended++;
		}
		else
		{
			scheduled[tasks[i]] = false;
			totalBatteryScheduled += Robot::maximumBattery;
			if (timeTst)
				deadlineAttended++;

			if (batteryTst)
				batteryAttended++;

			if (payloadTst)
				payloadAttended++;
		}
	}
	//Falta distancia até o depósito

	double payloadFactor = 1.0 + taskList->size() - payloadAttended;
	double payloadPart = std::pow((1 + totalPayload) / (1 + totalPayloadScheduled), payloadFactor);

	double energyFactor = 1.0 + taskList->size() - batteryAttended;
	double energypart = std::pow(2.0 + std::abs(totalBatteryScheduled - totalBatteryInTask), energyFactor) - 1.0;

	double timeFactor = 1.0 + taskList->size() - deadlineAttended;
	fitness = payloadPart * energypart * timeFactor;
}

void Chromosome::initialize()
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<uint16_t> robotDist(0, robotList->size() - 1);
	std::uniform_int_distribution<uint16_t> tasktDist(0, taskList->size() - 1);

	uint16_t j = robotDist(gen);
	uint16_t k = tasktDist(gen);
	for (uint16_t i = 0; i < taskList->size(); i++)
	{
		robots.push_back(j);
		j = (j + 1) % robotList->size();

		tasks.push_back(k);
		k = (k + 1) % taskList->size();

		scheduled.push_back(false);
	}

	std::shuffle(tasks.begin(), tasks.end(), gen);
	std::shuffle(robots.begin(), robots.end(), gen);

	computeFitness();
}

std::vector<uint16_t> &Chromosome::getTasks()
{
	return tasks;
}

std::vector<uint16_t> &Chromosome::getRobots()
{
	return robots;
}

std::vector<bool> &Chromosome::getScheduled()
{
	return scheduled;
}

double Chromosome::getFitness() const
{
	return fitness;
}

std::vector<ChromosomeResult> Chromosome::getResult()
{
	std::vector<ChromosomeResult> result;

	for (uint16_t i = 0; i < tasks.size(); i++)
		result.push_back(ChromosomeResult(robotList->at(robots[i]).getId(), taskList->at(tasks[i]).getId(), scheduled[tasks[i]]));

	return result;
}

bool Chromosome::allScheduled()
{
	for (auto tst : scheduled)
	{
		if (!tst)
			return false;
	}
	return true;
}

void Chromosome::printResult()
{
	std::cout << std::setw(12) << "Robots: ";
	for (uint16_t i = 0; i < robots.size(); i++)
	{
		std::cout << std::setw(3) << robotList->at(robots[i]).getId();
	}
	std::cout << std::endl
			  << std::setw(12) << "Tasks: ";
	for (uint16_t i = 0; i < tasks.size(); i++)
	{
		std::cout << std::setw(3) << taskList->at(tasks[i]).getId();
	}
	std::cout << std::endl
			  << std::setw(12) << "Failed: ";
	for (uint16_t i = 0; i < scheduled.size(); i++)
	{
		if (!scheduled[tasks[i]])
			std::cout << std::setw(3) << taskList->at(tasks[i]).getId();
	}
	std::cout << std::endl;
	std::cout << std::setw(12) << "Fitness: " << fitness << std::endl;
}

void Chromosome::getResult(std::vector<uint32_t> &tasksScheduled, std::vector<uint32_t> &robotsScheduled, std::vector<uint32_t> &tasksFailed)
{
	for (uint16_t i = 0; i < taskList->size(); i++)
	{
		if (scheduled[i])
		{
			tasksScheduled.push_back(taskList->at(tasks[i]).getId());
			robotsScheduled.push_back(robotList->at(robots[i]).getId());
		}
		else
		{
			tasksFailed.push_back(taskList->at(tasks[i]).getId());
		}		
	}	
}

double Chromosome::totalEnergy(bool allTasks)
{
	double totalEnergy = 0;
	std::vector<Robot> robotsCopy;
	std::copy(robotList->begin(), robotList->end(), std::back_inserter(robotsCopy));
	for (uint16_t i = 0; i < taskList->size(); i++)
	{
		if (allTasks || scheduled[i])
		{
			auto task = taskList->at(tasks[i]);
			auto &robot = robotsCopy[robots[i]];

			//Computing time and battery requirements
			double distanceToTheTask = distanceMaxtrix->at(robot.getCurrentLocation()).at(task.getPickUpLocation());
			double distanceInTask = distanceMaxtrix->at(task.getPickUpLocation()).at(task.getDeliveryLocation());
			double distance = distanceToTheTask + distanceInTask;
			double time = robot.computeTimeRequirement(distance);
			totalEnergy += robot.computeBatteryRequirement(time);

			robot.setCurrentLocation(task.getDeliveryLocation());
		}
	}
	return totalEnergy;
}

std::vector<double> Chromosome::totalEnergyByRobot(bool normalized, bool allTasks)
{
	std::vector<double> energyByRobot(robotList->size(), 0.0);
	std::vector<Robot> robotsCopy;
	std::copy(robotList->begin(), robotList->end(), std::back_inserter(robotsCopy));
	for (uint16_t i = 0; i < taskList->size(); i++)
	{
		if (allTasks || scheduled[i])
		{
			auto task = taskList->at(tasks[i]);
			auto &robot = robotsCopy[robots[i]];

			//Computing time and battery requirements
			double distanceToTheTask = distanceMaxtrix->at(robot.getCurrentLocation()).at(task.getPickUpLocation());
			double distanceInTask = distanceMaxtrix->at(task.getPickUpLocation()).at(task.getDeliveryLocation());
			double distance = distanceToTheTask + distanceInTask;
			double time = robot.computeTimeRequirement(distance);
			if (normalized)
				energyByRobot[robots[i]] += robot.computeBatteryRequirement(time) / robot.getUtilRemainingBattery();
			else
				energyByRobot[robots[i]] += robot.computeBatteryRequirement(time);

			robot.setCurrentLocation(task.getDeliveryLocation());
		}
	}
	return energyByRobot;
}

std::vector<double> Chromosome::totalEnergyByTask(bool allTasks)
{
	std::vector<double> energyByTask(taskList->size());
	std::vector<Robot> robotsCopy;
	std::copy(robotList->begin(), robotList->end(), std::back_inserter(robotsCopy));
	for (uint16_t i = 0; i < taskList->size(); i++)
	{
		if (allTasks || scheduled[i])
		{
			auto task = taskList->at(tasks[i]);
			auto &robot = robotsCopy[robots[i]];

			//Computing time and battery requirements
			double distanceToTheTask = distanceMaxtrix->at(robot.getCurrentLocation()).at(task.getPickUpLocation());
			double distanceInTask = distanceMaxtrix->at(task.getPickUpLocation()).at(task.getDeliveryLocation());
			double distance = distanceToTheTask + distanceInTask;
			double time = robot.computeTimeRequirement(distance);
			energyByTask[tasks[i]] = robot.computeBatteryRequirement(time);

			robot.setCurrentLocation(task.getDeliveryLocation());
		}
	}
	return energyByTask;
}

double Chromosome::totalTime(bool allTasks)
{
	double totalTime = 0;
	std::vector<Robot> robotsCopy;
	std::copy(robotList->begin(), robotList->end(), std::back_inserter(robotsCopy));
	for (uint16_t i = 0; i < taskList->size(); i++)
	{
		if (allTasks || scheduled[i])
		{
			auto task = taskList->at(tasks[i]);
			auto &robot = robotsCopy[robots[i]];

			//Computing time and battery requirements
			double distanceToTheTask = distanceMaxtrix->at(robot.getCurrentLocation()).at(task.getPickUpLocation());
			double distanceInTask = distanceMaxtrix->at(task.getPickUpLocation()).at(task.getDeliveryLocation());
			double distance = distanceToTheTask + distanceInTask;
			totalTime += robot.computeTimeRequirement(distance);

			robot.setCurrentLocation(task.getDeliveryLocation());
		}
	}
	return totalTime;
}

std::vector<double> Chromosome::totalTimeByRobot(bool allTasks)
{
	std::vector<double> totalTimeByRobot(robotList->size(), 0);
	std::vector<Robot> robotsCopy;
	std::copy(robotList->begin(), robotList->end(), std::back_inserter(robotsCopy));
	for (uint16_t i = 0; i < taskList->size(); i++)
	{
		if (allTasks || scheduled[i])
		{
			auto task = taskList->at(tasks[i]);
			auto &robot = robotsCopy[robots[i]];

			//Computing time and battery requirements
			double distanceToTheTask = distanceMaxtrix->at(robot.getCurrentLocation()).at(task.getPickUpLocation());
			double distanceInTask = distanceMaxtrix->at(task.getPickUpLocation()).at(task.getDeliveryLocation());
			double distance = distanceToTheTask + distanceInTask;
			totalTimeByRobot[robots[i]] += robot.computeTimeRequirement(distance);

			robot.setCurrentLocation(task.getDeliveryLocation());
		}
	}
	return totalTimeByRobot;
}

std::vector<double> Chromosome::totalTimeByTask(bool allTasks)
{
	std::vector<double> totalTimeByTask(taskList->size());
	std::vector<Robot> robotsCopy;
	std::copy(robotList->begin(), robotList->end(), std::back_inserter(robotsCopy));
	for (uint16_t i = 0; i < taskList->size(); i++)
	{
		if (allTasks || scheduled[i])
		{
			auto task = taskList->at(tasks[i]);
			auto &robot = robotsCopy[robots[i]];

			//Computing time and battery requirements
			double distanceToTheTask = distanceMaxtrix->at(robot.getCurrentLocation()).at(task.getPickUpLocation());
			double distanceInTask = distanceMaxtrix->at(task.getPickUpLocation()).at(task.getDeliveryLocation());
			double distance = distanceToTheTask + distanceInTask;
			totalTimeByTask[tasks[i]] = robot.computeTimeRequirement(distance);

			robot.setCurrentLocation(task.getDeliveryLocation());
		}
	}
	return totalTimeByTask;
}

uint64_t Chromosome::totalPayload(bool allTasks)
{
	uint64_t totalPayload = 0;
	for (uint16_t i = 0; i < taskList->size(); i++)
	{
		if (allTasks || scheduled[i])
		{
			auto task = taskList->at(tasks[i]);
			totalPayload += task.getPayload();
		}
	}
	return totalPayload;
}

std::vector<double> Chromosome::totalPayloadByRobot(bool normalized, bool allTasks)
{
	std::vector<double> totalPayloadByRobot(robotList->size(), 0);
	std::vector<uint16_t> totalTasksByRobot(robotList->size(), 0);
	for (uint16_t i = 0; i < taskList->size(); i++)
	{
		if (allTasks || scheduled[i])
		{
			auto task = taskList->at(tasks[i]);
			auto robot = robotList->at(robots[i]);
			if (normalized)
				totalPayloadByRobot[robots[i]] += (double)task.getPayload() / robot.getMaximumPayload();
			else
				totalPayloadByRobot[robots[i]] += task.getPayload();

			totalTasksByRobot[robots[i]] += 1;
		}
	}

	for (uint16_t i = 0; i < robotList->size(); i++)
	{
		if (totalTasksByRobot[i] > 0)
			totalPayloadByRobot[i] = totalPayloadByRobot[i] / totalTasksByRobot[i];
		else
			totalPayloadByRobot[i] = 0;
	}

	return totalPayloadByRobot;
}

uint16_t Chromosome::numberOfScheduled()
{
	uint16_t count = 0;
	for (bool s : scheduled)
	{
		if (s)
			count++;
	}
	return count;
}
