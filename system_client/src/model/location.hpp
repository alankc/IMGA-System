#ifndef LOCATION_HPP
#define LOCATION_HPP

#include <string>

class Location
{
	private:
		uint32_t id;
		uint32_t x;
		uint32_t y;
		double a;
		std::string description; 
		bool isDepot;

	public:
		Location(uint32_t id = 0, uint32_t x = 0, uint32_t y = 0, std::string description = "Empty", bool isDepot = false);
		void setId(uint32_t id);
		void setX(uint32_t x);
		void setY(uint32_t y);
		void setA(double a);
		void setDescription(std::string description);
		void setIsDepot(bool isDepot);

		uint32_t getId();
		uint32_t getX();
		uint32_t getY();
		double getA();
		std::string getDescription();
		bool getIsDepot();		

		friend bool operator== (const Location &p1, const Location &p2);
		bool operator< (const Location &locationObj);	
};

#endif