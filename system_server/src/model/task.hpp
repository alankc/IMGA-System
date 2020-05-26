#ifndef TASK_HPP
#define TASK_HPP

#include <cstdint>
#include <iostream>
#include <string>

class Task
{
private:
	uint32_t id;
	std::string description;
	std::string status;
	uint32_t pickUpLocation;
	uint32_t deliveryLocation;
	uint16_t payload;
	double deadline;
	uint32_t robotInCharge;
	uint32_t seqNumber;
public:
	Task(uint32_t id = 0, uint32_t pickUpLocation = 0, uint32_t deliveryLocation = 0, uint16_t payload = 0, double deadline = 0);

	void setId(uint32_t id);
	void setDescription(std::string description);
	void setStatus(std::string status);
	void setPickUpLocation(uint32_t pickUpLocation);
	void setDeliveryLocation(uint32_t deliveryLocation);
	void setPayload(uint16_t payload);
	void setDeadline(double deadline);
	void setRobotInCharge(uint32_t robotInCharge);
	void setSeqNumber(uint32_t seqNumber);

	uint32_t getId() const;
	std::string getDescription() const;
	std::string getStatus() const;
	uint32_t getPickUpLocation() const;
	uint32_t getDeliveryLocation() const;
	uint16_t getPayload() const;
	double getDeadline() const;
	uint32_t getRobotInCharge() const;
	uint32_t getSeqNumber() const;

	friend bool operator==(const Task &t1, const Task &t2);
	friend std::ostream &operator<<(std::ostream &os, const Task &t);
	bool operator<(const Task &taskObj);
};

#endif