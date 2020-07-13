#include <ros/ros.h>
#include "dao/robotDao.hpp"

int main(int argc, char **argv)
{
  std::string host = "localhost";
  std::string user = "root";
  std::string pass = "281094";
  std::string db = "ServerDB";

  GeneralDao gd(host, user, pass, db);
  RobotDao rd(&gd);
  Robot r;
  rd.getRobot(0, r);

  std::cout << r.getDescription() << std::endl;

  return 0;
}