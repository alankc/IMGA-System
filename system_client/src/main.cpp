#include <ros/ros.h>
#include <tf/tf.h>
#include "geometry_msgs/Twist.h"

#include "controller/generalController.hpp"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

double poseAMCLx, poseAMCLy, poseAMCLa;
void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msgAMCL)
{
  poseAMCLx = msgAMCL->pose.pose.position.x;
  poseAMCLy = msgAMCL->pose.pose.position.y;

  tf::Quaternion q(msgAMCL->pose.pose.orientation.x,
                   msgAMCL->pose.pose.orientation.y,
                   msgAMCL->pose.pose.orientation.z,
                   msgAMCL->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  poseAMCLa = yaw;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle vel;
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

  /*GeneralController gc;
  gc.run();*/

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
    std::cout << "deu ruim" << std::endl;

  return 0;
}