#include "task.hpp"

Task::Task(uint32_t id, uint32_t pickUpLocation, uint32_t deliveryLocation, uint16_t payload, double deadline) 
{
	this->id = id;
	this->pickUpLocation = pickUpLocation;
	this->deliveryLocation = deliveryLocation;
	this->payload = payload;
	this->deadline = deadline;
}

void Task::setId(uint32_t id)
{
	this->id = id;
}

void Task::setPickUpLocation(uint32_t pickUpLocation)
{
	this->pickUpLocation = pickUpLocation;
}

void Task::setDeliveryLocation(uint32_t deliveryLocation)
{
	this->deliveryLocation = deliveryLocation;
}

void Task::setPayload(uint16_t payload)
{
	this->payload = payload;
}

void Task::setDeadline(double deadline)
{
	this->deadline = deadline;
}

uint32_t Task::getId() const
{
	return id;
}

uint32_t Task::getPickUpLocation() const
{
	return pickUpLocation;
}

uint32_t Task::getDeliveryLocation() const
{
	return deliveryLocation;
}

uint16_t Task::getPayload() const
{
	return payload;
}

double Task::getDeadline() const
{
	return deadline;
}

bool operator== (const Task& t1, const Task &t2)
{
	return t1.id == t2.id;
}

bool Task::operator< (const Task &taskObj)
{
	return (this->id < taskObj.id);
}