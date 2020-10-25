#include "robot.hpp"

double Robot::maximumBattery = 100.0;
const std::string Robot::STATUS_FREE = "FR";
const std::string Robot::STATUS_WORKING = "W";
const std::string Robot::STATUS_TO_DEPOT = "D";
const std::string Robot::STATUS_FAIL = "FA";
const std::string Robot::STATUS_CHARGING = "C";

Robot::Robot(uint32_t id, uint32_t depot, uint32_t currentLocation, uint32_t maximumPayload, double remainingBattery, double dischargeFactor, double batteryThreshold, double mediumVelocity)
{
	this->id = id;
	this->depot = depot;
	this->currentLocation = currentLocation;
	this->maximumPayload = maximumPayload;
	this->remainingBattery = remainingBattery;
	this->dischargeFactor = dischargeFactor;
	this->batteryThreshold = batteryThreshold;
	this->mediumVelocity = mediumVelocity;
}

void Robot::setId(uint32_t id)
{
	this->id = id;
}

void Robot::setDescription(std::string description)
{
	this->description = description;
}

void Robot::setStatus(std::string status)
{
	this->status = status;
}

void Robot::setDepot(uint32_t depot)
{
	this->depot = depot;
}

void Robot::setCurrentLocation(uint32_t currentLocation)
{
	this->currentLocation = currentLocation;
}

void Robot::setMaximumPayload(uint32_t maximumPayload)
{
	this->maximumPayload = maximumPayload;
}

void Robot::setRemainingBattery(double remainingBattery)
{
	this->remainingBattery = remainingBattery;
}

void Robot::setDischargeFactor(double dischargeFactor)
{
	this->dischargeFactor = dischargeFactor;
}

void Robot::setBatteryThreshold(double batteryThreshold)
{
	this->batteryThreshold = batteryThreshold;
}

void Robot::setMediumVelocity(double mediumVelocity)
{
	this->mediumVelocity = mediumVelocity;
}

void Robot::updateMediumVelocity(double distance, double time)
{
	double tmpMedVel = distance / time;

	if (tmpMedVel < this->mediumVelocity)
		this->mediumVelocity = distance / time;
}

uint32_t Robot::getId() const
{
	return id;
}

std::string Robot::getDescription() const
{
	return description;
}

std::string Robot::getStatus() const
{
	return status;
}

uint32_t Robot::getDepot() const
{
	return depot;
}

uint32_t Robot::getCurrentLocation() const
{
	return currentLocation;
}

uint32_t Robot::getMaximumPayload() const
{
	return maximumPayload;
}

double Robot::getRemainingBattery() const
{
	return remainingBattery;
}

double Robot::getUtilRemainingBattery() const
{
	return remainingBattery - batteryThreshold;
}

double Robot::getDischargeFactor() const
{
	return dischargeFactor;
}

double Robot::getBatteryThreshold() const
{
	return batteryThreshold;
}

double Robot::getMediumVelocity() const
{
	return mediumVelocity;
}

double Robot::computeTimeRequirement(double distanceInMeters)
{
	return (distanceInMeters / mediumVelocity);
}

double Robot::computeBatteryRequirement(double timeInSeconds)
{
	return timeInSeconds * dischargeFactor;
}

bool operator==(const Robot &r1, const Robot &r2)
{
	return r1.id == r2.id;
}

bool Robot::operator<(const Robot &robotObj)
{
	return this->id < robotObj.id;
}

std::ostream &operator<<(std::ostream &os, const Robot &r)
{
	os << r.id << " ";
	os << r.description << " ";
	os << r.status << " ";
	os << r.depot << " ";
	os << r.currentLocation << " ";
	os << r.maximumPayload << " ";
	os << r.remainingBattery << " ";
	os << r.dischargeFactor << " ";
	os << r.batteryThreshold << " ";
	os << r.mediumVelocity << " ";
	
	return os;
}