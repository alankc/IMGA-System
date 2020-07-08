#include <ros/ros.h>
#include "navigation/navigation.hpp"
#include <system_client/MsgTaskList.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  
  Navigation n("robot1", "move_base", "map");

  n.navigateTo(0.0, 0.0, 1.0);

  n.navigateTo(-1.0, 1.0, 1.0);
  return 0;
}