#include <ros/ros.h>
#include "controller/generalController.hpp"

int main(int argc, char **argv)
{
  //ros::init(argc, argv, "talker");
  
  GeneralController gc;
  gc.run();
  return 0;
}