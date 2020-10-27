#include <ros/ros.h>
#include <system_server/MsgRequest.h>
#include <system_server/MsgRobotData.h>
#include <iostream>

void callback_RE(const system_server::MsgRequest &msg)
{
    std::string s = std::to_string(msg.type) + "--" + std::to_string(msg.data);
    std::cout << s << std::endl;
}

void callback_RD(const system_server::MsgRobotData &msg)
{
    std::string s = "Robot: " + std::to_string(msg.id) + "--" + std::to_string(msg.battery);
    s += "--" + std::to_string(msg.minSpeed);
    s += "--" + std::to_string(msg.currLocation);
    s += "--" + msg.status;
    std::cout << s << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "echo");
    ros::NodeHandle nh;
    ros::Subscriber subTask = nh.subscribe("server_request", 10, callback_RE);
    ros::Subscriber subRobt = nh.subscribe("server_robot_data", 10, callback_RD);
    ros::spin();

    return 0;
}