#ifndef GA_HPP
#define GA_HPP

#include "chromosome.hpp"
#include <cstdint>
#include <vector>
#include <functional>

struct _GAParameters;
typedef struct _GAParameters GAParameters;

class GA
{
	public:
		enum SelectionMode
		{
			rouletteWheel = 0,
			rank,
			tournament
		};

		enum CrossoverMode
		{
			order = 0,
			orderBased,
			positionBased
		};

		enum MutationMode
		{
			exchange = 0,
			displacement,
			inversion,
			scramble
		};
			
		GA(/* args */);
		GA(GAParameters gaParameters);
		~GA();
		void initialize();
		void solve();
		Chromosome getBest();
		std::vector<Chromosome>& getPopulation();
		void printPopulation();	
	private:
		//paramters
		SelectionMode selectionMode;
		CrossoverMode crossoverMode;
		MutationMode mutationMode;
		uint16_t populationSize;
		uint64_t maxIterations;
		double goalFitness;
		uint64_t noChangeLimit;
		double elitismRate;
		double mutationRate;

		//ga solver variables
		std::vector<Chromosome> population;
		std::vector<Chromosome> tempPopulation;
		uint16_t c1;
		uint16_t c2;
		double totalFitness;
		std::function<void()> selectionFunction;
		std::function<void()> acumulateTotalFitnessFunction;
		std::function<Chromosome(Chromosome &c1, Chromosome &c2)> crossoverFunction;
		std::function<void(Chromosome &c)> mutationFunction;

		Chromosome best;

		void rouletteWheelSelection();
		void rouletteWheelAcumulateTotalFitness();
		
		void rankSelection();
		void rankAcumulateTotalFitness();
		
		void tournamentSelection(uint16_t size);
		void tournamentAcumulateTotalFitness();
};

struct _GAParameters
{
	GA::SelectionMode selectionMode = GA::SelectionMode::tournament;
	GA::CrossoverMode crossoverMode = GA::CrossoverMode::orderBased;
	GA::MutationMode mutationMode = GA::MutationMode::displacement;
	uint16_t populationSize = 100;
	uint64_t maxIterations = 50000;
	uint64_t noChangeLimit = 5000;
	double goalFitness = 1.0;
	double elitismRate = 0.25;
	double mutationRate = 0.25;
} ;

#endif