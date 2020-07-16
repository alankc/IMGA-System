#include <ros/ros.h>
#include "controller/generalController.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  //GeneralController gc;
  //gc.run();

  Navigator n("robot1");
  n.start();
  n.navigateTo(0.1, 0.2, 0.6);

  ros::Rate r(1);
  while (n.stillNavigating())
  {
    ros::spinOnce();
    r.sleep();
  }

  if (n.hasArrived())
    std::cout << "hasArrived" << std::endl;
  else
    std::cout << "deu ruim" << std::endl;

  return 0;
}