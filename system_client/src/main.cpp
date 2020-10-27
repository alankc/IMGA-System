#include <ros/ros.h>
#include "controller/generalController.hpp"
#include "battery/batterySimulator.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "system_client");
  ros::NodeHandle nh;

  int32_t robot_id;
  std::string host;
  std::string user;
  std::string pass;
  std::string db;
  std::string navigator_topic;
  std::string navigator_frame;
  std::string server_request;
  std::string server_robot_data;
  double battery_start;
  double battery_noise;

  ros::spinOnce();

  nh.param<int32_t>("system_client/robot_id", robot_id, 0);
  nh.param<std::string>("system_client/host", host, "localhost");
  nh.param<std::string>("system_client/user", user, "root");
  nh.param<std::string>("system_client/pass", pass, "281094");
  nh.param<std::string>("system_client/db", db, "ServerDB");
  nh.param<std::string>("system_client/navigator_topic", navigator_topic, "move_base");
  nh.param<std::string>("system_client/navigator_frame", navigator_frame, "map");
  nh.param<std::string>("system_client/server_request", server_request, "/server_request");
  nh.param<std::string>("system_client/server_robot_data", server_robot_data, "/server_robot_data");
  nh.param<double>("system_client/battery_start", battery_start, 100.0);
  nh.param<double>("system_client/battery_noise", battery_noise, 0.001);


  GeneralController gc(robot_id, host, user, pass, db, navigator_topic, navigator_frame, server_request, server_robot_data, battery_start, battery_noise);
  gc.run();

  return 0;
}