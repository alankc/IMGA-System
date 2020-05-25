#ifndef TASK_HPP
#define TASK_HPP

#include <cstdint>

class Task
{
private:
	uint32_t id;
	uint32_t pickUpLocation;
	uint32_t deliveryLocation;
	uint16_t payload;
	double deadline;

public:
	Task(uint32_t id = 0, uint32_t pickUpLocation = 0, uint32_t deliveryLocation = 0, uint16_t payload = 0, double deadline = 0);

	void setId(uint32_t id);
	void setPickUpLocation(uint32_t pickUpLocation);
	void setDeliveryLocation(uint32_t deliveryLocation);
	void setPayload(uint16_t payload);
	void setDeadline(double deadline);

	uint32_t getId() const;
	uint32_t getPickUpLocation() const;
	uint32_t getDeliveryLocation() const;
	uint16_t getPayload() const;
	double getDeadline() const;

	friend bool operator== (const Task& t1, const Task &t2);
	bool operator< (const Task &taskObj);
};

#endif