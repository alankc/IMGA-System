#include <ros/ros.h>
#include "controller/generalController.hpp"
#include "battery/batterySimulator.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;

  /*ros::NodeHandle vel;
  ros::Publisher vel_pub = vel.advertise<geometry_msgs::Twist>("/robot1/cmd_vel", 100);
  geometry_msgs::Twist msg;
  msg.linear.x = 0.0;
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = 0.1;

  ros::NodeHandle amcl;
  ros::Subscriber sub_amcl = amcl.subscribe("/robot1/amcl_pose", 100, poseAMCLCallback);

  Navigator n("robot1");
  n.start();

  ros::Rate r(1);

  n.navigateTo(-19.0, -12.5, 1.570796327);
  while (n.stillNavigating() && ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  r.sleep();
  r.sleep();
  r.sleep();

  n.navigateTo(-19.0, -12.5, -1.570796327);
  while (n.stillNavigating() && ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  r.sleep();
  r.sleep();
  r.sleep();

  n.navigateTo(-10.0, -3.5, 1.570796327);
  while (n.stillNavigating() && ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  if (n.hasArrived())
    std::cout << "hasArrived" << std::endl;
  else
    std::cout << "deu ruim" << std::endl;*/

  //GeneralController gc;
  //gc.run();

  BatterySimulator bS(30, 5, 0.1);
  ros::Rate r(2);
  uint16_t i = 0;
  double batt = bS.getRemaningBattery();

  while (batt >= 0 && ros::ok())
  {
    std::cout << i++ << "---" << batt << std::endl;
    r.sleep();
    batt = bS.getRemaningBattery();
  }
  

  return 0;
}