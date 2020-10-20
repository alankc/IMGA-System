#include <ros/ros.h>
#include <system_client/MsgRequest.h>
#include <iostream>

void callback(const system_client::MsgRequest &msg)
{
    std::string s = std::to_string(msg.type) + "--" + std::to_string(msg.data);
    std::cout << s << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "echo");
    ros::NodeHandle nh;
    ros::Subscriber subTask = nh.subscribe("rqt123", 10, callback);
    ros::spin();

    return 0;
}