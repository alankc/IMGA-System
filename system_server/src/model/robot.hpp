#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <cstdint>
#include <vector>
#include <string>

class Robot
{
private:
	uint32_t id;
	std::string description;
	std::string status;
	uint32_t depot;
	uint32_t currentLocation;
	uint32_t maximumPayload;
	double remainingBattery;
	double dischargeFactor;
	double batteryThreshold;
	double mediumVelocity;
	uint32_t waitingTime;

public:
	static double maximumBattery;
	Robot(uint32_t id = 0, uint32_t depot = 0, uint32_t currentLocation = 0, uint32_t maximumPayload = 0, double remainingBattery = 0, double dischargeFactor = 0, double batteryThreshold = 0, double mediumVelocity = 0);

	void setId(uint32_t id);
	void setDescription(std::string description);
	void setStatus(std::string status);
	void setDepot(uint32_t depot);
	void setCurrentLocation(uint32_t currentLocation);
	void setMaximumPayload(uint32_t maximumPayload);
	void setRemainingBattery(double remainingBattery);
	void setDischargeFactor(double dischargeFactor);
	void setBatteryThreshold(double batteryThreshold);
	void setMediumVelocity(double mediumVelocity);
	void setWaitingTime(uint32_t waitingTime);

	uint32_t getId() const;
	std::string getDescription() const;
	std::string getStatus() const;
	uint32_t getDepot() const;
	uint32_t getCurrentLocation() const;
	uint32_t getMaximumPayload() const;
	double getRemainingBattery() const;
	double getUtilRemainingBattery() const;
	double getDischargeFactor() const;
	double getBatteryThreshold() const;
	double getMediumVelocity() const;
	uint32_t getWaitingTime() const;

	double computeTimeRequirement(double distanceInMeters);
	double computeBatteryRequirement(double timeInSeconds);

	friend bool operator==(const Robot &r1, const Robot &r2);
	bool operator<(const Robot &robotObj);

	static const std::string STATUS_FREE;
	static const std::string STATUS_WORKING;
	static const std::string STATUS_TO_DEPOT;
	static const std::string STATUS_FAIL;
	static const std::string STATUS_CHARGING;
};

#endif