#include "locationController.hpp"
#include <random>

LocationController::LocationController() {}

LocationController::LocationController(GeneralDao *gDao)
{
    this->ld = LocationDao(gDao);
    this->currX = 0;
    this->currY = 0;
    this->currA = 0;
    this->planDone = true;
}

LocationController::~LocationController()
{
}

void LocationController::run()
{
    this->pubPosition = nh.advertise<geometry_msgs::PoseStamped>("/server_planner/goal", 10);
    this->subPlan = nh.subscribe("/server_planner/planner/plan", 100, &LocationController::callbackPlanner, this);
    this->threadServerTransform = new std::thread(&LocationController::loopTransform, this);
}

void LocationController::sendServerTransform()
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    //Needs to be negative, i do not know why!!
    transform.setOrigin(tf::Vector3(-this->currX, -this->currY, 0.0));
    tf::Quaternion q = tf::createQuaternionFromYaw(0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "server_frame", "map"));
    ros::spinOnce();
}

void LocationController::sendGoalPosition()
{
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = this->goalX;
    msg.pose.position.y = this->goalY;
    msg.pose.position.z = 0;
    tf::Quaternion qt = tf::createQuaternionFromYaw(0);
    msg.pose.orientation.x = qt.getX();
    msg.pose.orientation.y = qt.getY();
    msg.pose.orientation.z = qt.getZ();
    msg.pose.orientation.w = qt.getW();
    pubPosition.publish(msg);
    ros::spinOnce();
}

void LocationController::callbackPlanner(const nav_msgs::Path &msg)
{
    auto &poses = msg.poses;
    geometry_msgs::PoseStamped pBefore, pCurr;
    pBefore = poses[0];
    pathLength = 0;
    //Testing if the path start with the current position
    //Some times the server transform has delay to arrive to ros
    bool tst = (std::abs(currX - pBefore.pose.position.x) < 0.01) && (std::abs(currY - pBefore.pose.position.y) < 0.01);
    if (!planDone && tst)
    {
        for (uint32_t i = 1; i < poses.size(); i++)
        {
            pCurr = poses[i];
            double dx = pCurr.pose.position.x - pBefore.pose.position.x;
            double dy = pCurr.pose.position.y - pBefore.pose.position.y;
            pathLength += std::sqrt(dx * dx + dy * dy);
            pBefore = poses[i];
        }
        this->planDone = true;
    }
    else //case not update
    {
        sendGoalPosition();
    }
}

void LocationController::loopTransform()
{
    ros::Rate r(1); //the node maximum time limit in 10
    while (ros::ok())
    {
        sendServerTransform();
        ros::spinOnce();
        r.sleep();
    }
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
    {
        distanceMatrix[i].resize(maxId + 1);
        for (uint32_t j = 0; j < maxId + 1; j++)
        {
            distanceMatrix[i][j] = 0;
        }
    }

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

void LocationController::computeDistanceMatrix(bool justMissingLocations)
{
    ros::Rate r(50);
    for (uint32_t l1 = 0; l1 < locationList.size(); l1++)
    {
        for (uint32_t l2 = 0; l2 < locationList.size(); l2++)
        {
            if (!ros::ok())
                return;

            if (l1 != l2)
            {
                double &currDistance = distanceMatrix[locationList[l1].getId()][locationList[l2].getId()];
                if (!justMissingLocations || (justMissingLocations && currDistance == 0))
                {
                    //Seting server position
                    this->currX = locationList[l1].getX();
                    this->currY = locationList[l1].getY();
                    this->currA = locationList[l1].getA();
                    sendServerTransform();
                    r.sleep();

                    //Seting goal position and making plan
                    this->planDone = false;
                    this->goalX = locationList[l2].getX();
                    this->goalY = locationList[l2].getY();
                    this->goalA = locationList[l2].getA();
                    sendGoalPosition();
                    r.sleep();

                    //waiting update, is not the best way, but ok...
                    ros::Rate rCheckPlan(100);
                    while (!this->planDone && ros::ok())
                    {
                        rCheckPlan.sleep();
                        ros::spinOnce();
                    }
                    currDistance = pathLength;
                    std::cout << locationList[l1].getId() << ","
                              << locationList[l2].getId() << ","
                              << currDistance << std::endl;
                }
            }
        }
    }
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