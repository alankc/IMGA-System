#include "locationController.hpp"
#include <random>

LocationController::LocationController() {}

LocationController::LocationController(GeneralDao *gDao)
{
    this->ld = LocationDao(gDao);
}

LocationController::~LocationController()
{
}

void LocationController::updateLocationList()
{
    locationList.clear();
    bool tst = ld.getLocationList(locationList);
    if (!tst)
        std::cout << "Fail to update location list" << std::endl;
    else
        std::cout << "Location list has been updated" << std::endl;
}

void LocationController::updateDistanceMatrix()
{
    auto it = std::max_element(locationList.begin(), locationList.end());
    uint32_t maxId = it->getId();
    distanceMatrix.clear();
    distanceMatrix.resize(maxId + 1);
    for (uint32_t i = 0; i < maxId + 1; i++)
        distanceMatrix[i].resize(maxId + 1);

    std::vector<DistanceUnit> dv;
    bool tst = ld.getDistanceTable(dv);
    if (tst)
    {
        for (auto d : dv)
        {
            distanceMatrix[d.from][d.to] = d.distance;
        }
        std::cout << "Distance matrix has been updated" << std::endl;
    }
    else
        std::cout << "Fail to update distance matrix" << std::endl;
}

Location *LocationController::getLocationById(uint32_t id, bool isDepot)
{
    if (isDepot)
    {
        auto it = std::find(depotList.begin(), depotList.end(), id);

        if (it != depotList.end())
            return &(*it);
    }
    else
    {
        auto it = std::find(locationList.begin(), locationList.end(), id);

        if (it != locationList.end())
            return &(*it);
    }

    return NULL;
}

Location *LocationController::getLocationByIndex(uint32_t index, bool isDepot)
{
    if (isDepot)
        return &depotList[index];
    else
        return &locationList[index];
}

std::size_t LocationController::getLocationListSize()
{
    return locationList.size();
}

std::size_t LocationController::getDepotListSize()
{
    return depotList.size();
}

void LocationController::setDistance(uint32_t from, uint32_t to, double distance)
{
    distanceMatrix[from][to] = distance;
}

double LocationController::getDistance(uint32_t from, uint32_t to)
{
    return distanceMatrix[from][to];
}

std::vector<std::vector<double>> *LocationController::getDistanceMatrix()
{
    return &distanceMatrix;
}

void LocationController::gerenateLocations(uint32_t numOfLocations, uint32_t numOfDepots, double minDistance, double maxDistance)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> randomDistance(minDistance, maxDistance);

    locationList.clear();
    for (uint32_t i = 0; i < numOfLocations; i++)
    {
        Location l;
        l.setId(i);
        l.setDescription("Location " + std::to_string(i));
        if (i < numOfDepots)
            l.setIsDepot(true);

        this->locationList.push_back(l);
    }

    distanceMatrix.clear();
    distanceMatrix.resize(numOfLocations);
    for (uint32_t i = 0; i < numOfLocations; i++)
        distanceMatrix[i].resize(numOfLocations);

    for (uint32_t i = 0; i < numOfLocations; i++)
    {
        for (uint32_t j = 0; j < numOfLocations; j++)
        {
            if (i == j)
            {
                distanceMatrix[i][j] = 0;
            }
            else if (locationList[i].getIsDepot() && locationList[j].getIsDepot())
            {
                distanceMatrix[i][j] = maxDistance;
            }
            else
            {
                distanceMatrix[i][j] = randomDistance(gen);
            }
            distanceMatrix[j][i] = distanceMatrix[i][j];
        }
    }
}