#include "ga.hpp"

#include <algorithm>
#include <random>
#include <iostream>

GA::GA(/* args */)
{
	populationSize = 150;
	maxIterations = 50000;
	goalFitness = 0.0;
	noChangeLimit = 5000;

	elitismRate = 0.2;
	mutationRate = 0.5;
}

GA::GA(GAParameters gaParameters)
{
	this->populationSize = gaParameters.populationSize;
	this->maxIterations = gaParameters.maxIterations;
	this->noChangeLimit = gaParameters.noChangeLimit;	
	this->goalFitness = gaParameters.goalFitness;
	this->elitismRate = gaParameters.elitismRate;
	this->mutationRate = gaParameters.mutationRate;
	this->selectionMode = gaParameters.selectionMode;
	this->crossoverMode = gaParameters.crossoverMode;
	this->mutationMode = gaParameters.mutationMode;

	switch (selectionMode)
	{
	case rouletteWheel:
		this->selectionFunction = [this]() { this->rouletteWheelSelection(); };
		this->acumulateTotalFitnessFunction = [this]() { this->rouletteWheelAcumulateTotalFitness(); };
		break;

	case rank:
		this->selectionFunction = [this]() { this->rankSelection(); };
		this->acumulateTotalFitnessFunction = [this]() { this->rankAcumulateTotalFitness(); };	
		break;

	case tournament:	
	default:
		this->selectionFunction = [this]() { this->tournamentSelection(2); };
		this->acumulateTotalFitnessFunction = [this]() { this->tournamentAcumulateTotalFitness(); };	
		break;
	}

	switch (crossoverMode)
	{
	case order:
		this->crossoverFunction = Chromosome::orderCrossover;
		break;

	case positionBased:
		this->crossoverFunction = Chromosome::positionBasedCrossover;
		break;

	case orderBased:			
	default:
		this->crossoverFunction = Chromosome::orderBasedCrossover;
		break;
	}

	switch (mutationMode)
	{
	case exchange:
		this->mutationFunction = Chromosome::exchangeMutation;
		break;

	case scramble:
		this->mutationFunction = Chromosome::scrambleMutation;
		break;	

	case inversion:
		this->mutationFunction = Chromosome::inversionMutation;
		break;

	case displacement:
	default:
		this->mutationFunction = Chromosome::displacementMutation;
		break;
	}
	
}

GA::~GA()
{
}

void GA::rouletteWheelSelection()
{
	static std::random_device rd;
	static std::mt19937 gen(rd());
	std::uniform_real_distribution<double> sizeDist(0, populationSize - 1);
	std::uniform_real_distribution<double> valueDist(0, totalFitness);

	double max = valueDist(gen);
	double s = 0;
	uint16_t i = sizeDist(gen);
	while (s < max)
	{	
		i = (i + 1) % populationSize;
		s += 1.0 / population[i].getFitness();		
	}	
	c1 = i;

	max = valueDist(gen);
	s = 0;
	i = sizeDist(gen);
	while (s < max)
	{	
		i = (i + 1) % populationSize;
		s += 1.0 / population[i].getFitness();
	}
	c2 = i;	

	if (population[c2].getFitness() < population[c1].getFitness())
		std::swap(c1, c2);	
}

void GA::rouletteWheelAcumulateTotalFitness()
{
	totalFitness = 0;
	for (uint16_t i = 0; i < populationSize; i++)
		totalFitness += 1.0 / population[i].getFitness();
}

void GA::rankSelection()
{
	static std::random_device rd;
	static std::mt19937 gen(rd());
	std::uniform_real_distribution<double> sizeDist(0, populationSize - 1);
	std::uniform_real_distribution<double> valueDist(0, totalFitness);

	double max = valueDist(gen);
	double s = 0;
	uint16_t i = sizeDist(gen);
	while (s < max)
	{	
		i = (i + 1) % populationSize;
		s += 1.0 / (i + 1);		
	}	
	c1 = i;

	max = valueDist(gen);
	s = 0;
	i = sizeDist(gen);
	while (s < max)
	{	
		i = (i + 1) % populationSize;
		s += 1.0 / (i + 1);	
	}
	c2 = i;	

	if (population[c2].getFitness() < population[c1].getFitness())
		std::swap(c1, c2);
}

void GA::rankAcumulateTotalFitness()
{
	totalFitness = 0;
	for (uint16_t i = 0; i < populationSize; i++)
		totalFitness += 1.0 / (i + 1);
}

void GA::tournamentSelection(uint16_t size)
{
	static std::random_device rd;
	static std::mt19937 gen(rd());
	std::uniform_int_distribution<uint16_t> sizeDist(0, populationSize - 1);

	c1 = sizeDist(gen);
	c2 = sizeDist(gen);

	for (uint16_t i = 0; i < size; i++)
	{
		uint16_t tempMaxSeq1 = sizeDist(gen);
		uint16_t tempMaxSeq2 = sizeDist(gen);

		if (population[tempMaxSeq1].getFitness() < population[c1].getFitness())
			c1 = tempMaxSeq1;

		if (population[tempMaxSeq2].getFitness() < population[c2].getFitness())
			c2 = tempMaxSeq2;	
	}

	if (population[c2].getFitness() < population[c1].getFitness())
		std::swap(c1, c2);
}

void GA::tournamentAcumulateTotalFitness()
{
	//Do nothing
}

void GA::initialize()
{
	population.clear();
	tempPopulation.clear();

	for (uint16_t i = 0; i < populationSize; i++)
	{
		Chromosome chrTmp(true);
		population.push_back(chrTmp);
	}
	std::sort(population.begin(), population.end());	
	best = population[0];
}

void GA::solve()
{
	const uint16_t elitismMaxIndex = std::round(populationSize * elitismRate);
	const uint16_t crossoverMaxIndex = std::round(populationSize * (1 - elitismRate));
	const uint16_t mutationMaxIndex = std::round(populationSize * mutationRate);

	//std::cout << elitismMaxIndex << " " << crossoverMaxIndex << " " << mutationMaxIndex << "\n";

	std::random_device rd;
	std::mt19937 gen(rd());
	//std::uniform_int_distribution<uint16_t> sizeDist(0, populationSize - 1);
	std::uniform_int_distribution<uint16_t> mutationDist(0, populationSize - 1);

	uint64_t iteration = 0;
	uint64_t noChangeCounter = 0;
	//goalFitness < population[0].getFitness() because it is executed several times
	//goalFitness < best.getFitness() work just in the first time
	//No nedd sort becaus population its sorted during the migration
	acumulateTotalFitnessFunction();
	while ((goalFitness < population[0].getFitness()) && (iteration < maxIterations) && (noChangeCounter < noChangeLimit))
	{
		//Elitism
		for (uint16_t i = 0; i < elitismMaxIndex; i++)
			tempPopulation.push_back(population[i]);		
			
		//Selection, Crossover
		for (uint16_t i = 0; i < crossoverMaxIndex; i++)
		{
			//Selection
			selectionFunction();

			//Crossover
			auto tmp = crossoverFunction(population[c1], population[c2]);		
			tempPopulation.push_back(tmp);
		}

		//Mutation
		for (uint16_t i = 0; i < mutationMaxIndex; i++)
			mutationFunction(tempPopulation[mutationDist(gen)]);

		population.clear();
		std::copy(tempPopulation.begin(), tempPopulation.end(), std::back_inserter(population));
		tempPopulation.clear();
		std::sort(population.begin(), population.end());
		acumulateTotalFitnessFunction();

		//Verifing best
		if (best.getFitness() > population[0].getFitness())
		{
			best = population[0];
			noChangeCounter = 0;
			//std::cout << "Best Change\n";
		}
		else
			noChangeCounter++; //add increment in mutation
		/*if (iteration % 100 == 0)
			std::cout << "Iteration: " << iteration << "\t"
					  << "Best Fitness: " << best.getFitness() << "\n";*/
		iteration++;
	}

	//std::cout << "\n*** THE END ***\n";
}

Chromosome GA::getBest()
{
	return best;
}

std::vector<Chromosome>& GA::getPopulation()
{
	return population;
}

void GA::printPopulation()
{
	std::cout << std::endl;
	for (uint16_t i = 0; i < population.size(); i++)
	{
		auto tasks = population[i].getTasks();
		auto robots = population[i].getRobots();
		std::cout << "Chromosome: " << i << "\nTasks: ";
		for (uint16_t j = 0; j < tasks.size(); j++)
		{
			std::cout << tasks[i] << " ";
		}
		std::cout << "\nRobots: ";
		for (uint16_t j = 0; j < robots.size(); j++)
		{
			std::cout << robots[i] << " ";
		}
		std::cout << "\n";
	}
	std::cout << "\n"
			  << std::endl;
}