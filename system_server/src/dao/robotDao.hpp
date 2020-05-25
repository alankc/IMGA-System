#ifndef ROBOT_DAO_HPP
#define ROBOT_DAO_HPP

#include "generalDao.hpp"
#include "../model/robot.hpp"

class RobotDao
{
private:
    GeneralDao *gDao;
    
public:
    RobotDao(GeneralDao *gDao);
    ~RobotDao();
    bool getRobotList(std::vector<Robot> &robotList);
};


#endif