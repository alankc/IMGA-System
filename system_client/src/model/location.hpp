#ifndef LOCATION_HPP
#define LOCATION_HPP

#include <string>

class Location
{
	private:
		uint32_t id;
		double x;
		double y;
		double a;
		std::string description; 
		bool isDepot;

	public:
		Location(uint32_t id = 0, double x = 0, double y = 0, std::string description = "Empty", bool isDepot = false);
		void setId(uint32_t id);
		void setX(double x);
		void setY(double y);
		void setA(double a);
		void setDescription(std::string description);
		void setIsDepot(bool isDepot);

		uint32_t getId();
		double getX();
		double getY();
		double getA();
		std::string getDescription();
		bool getIsDepot();		

		friend bool operator== (const Location &p1, const Location &p2);
		bool operator< (const Location &locationObj);	
};

#endif