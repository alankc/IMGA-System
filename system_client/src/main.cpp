#include <ros/ros.h>
#include "navigation/navigator.hpp"
#include <system_client/MsgTaskList.h>
#include "communication/communicator.hpp"

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::start();
  /*Navigator n("robot1", "move_base", "map");

  n.navigateTo(0.0, 0.0, 1.0);

  n.navigateTo(-1.0, 1.0, 1.0);*/
  Communicator c;
  c.run();
  
  ros::spin();
  return 0;
}