#include "location.hpp"

Location::Location(uint32_t id, uint32_t x, uint32_t y, std::string description, bool isDepot)
{
	this->id = id;
	this->x = x;
	this->y = y;
	this->description = description;
	this->isDepot = isDepot;
}

void Location::setId(uint32_t id)
{
	this->id = id;
}

void Location::setX(double x)
{
	this->x = x;
}

void Location::setY(double y)
{
	this->y = y;
}

void Location::setA(double a)
{
	this->y = a;
}

void Location::setDescription(std::string description)
{
	this->description = description;
}

void Location::setIsDepot(bool isDepot)
{
	this->isDepot = isDepot;	
}

uint32_t Location::getId()
{
	return id;
}

double Location::getX()
{
	return x;
}

double Location::getY()
{
	return y;
}

double Location::getA()
{
	return a;
}

std::string Location::getDescription()
{
	return description;
}

bool Location::getIsDepot()
{
	return isDepot;	
}

bool operator==(const Location &p1, const Location &p2)
{
	return p1.id == p2.id;
}

bool Location::operator<(const Location &robotObj)
{
	return this->id < robotObj.id;
}