#include <ros/ros.h>
#include "controller/generalController.hpp"
#include "battery/batterySimulator.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "system_client");
  ros::NodeHandle nh;

  std::string host;
  std::string user;
  std::string pass;
  std::string db;
  std::string navigator_topic;
  std::string navigator_frame;
  std::string server_request;
  std::string server_robot_data;
  double battery_start;

  ros::spinOnce();

  nh.param<std::string>("system_client/host", host, "localhost");
  nh.param<std::string>("system_client/user", user, "root");
  nh.param<std::string>("system_client/pass", pass, "281094");
  nh.param<std::string>("system_client/db", db, "ServerDB");
  nh.param<std::string>("system_client/navigator_topic", navigator_topic, "move_base");
  nh.param<std::string>("system_client/navigator_frame", navigator_frame, "map");
  nh.param<std::string>("system_client/server_request", server_request, "/server_request");
  nh.param<std::string>("system_client/server_robot_data", server_robot_data, "/server_robot_data");
  nh.param<double>("system_client/battery_start", battery_start, 100.0);

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

  GeneralController gc;
  gc.run();

  /*BatterySimulator bS(30, 5, 0.1);
  ros::Rate r(2);
  uint16_t i = 0;
  double batt = bS.getRemaningBattery();

  while (batt >= 0 && ros::ok())
  {
    std::cout << i++ << "---" << batt << std::endl;
    r.sleep();
    batt = bS.getRemaningBattery();
  }*/

  return 0;
}