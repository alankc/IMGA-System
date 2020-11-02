#include <ros/ros.h>
#include <system_client/MsgRequest.h>
#include <system_server/MsgRobotData.h>
#include <system_client/MsgTaskList.h>
#include <iostream>

void callback_RE(const system_client::MsgRequest &msg)
{
    std::string s = std::to_string(msg.type) + "--" + std::to_string(msg.data);
    std::cout << s << std::endl;
}

void callback_TL(const system_client::MsgTaskList &msg)
{
    for (uint32_t i = 0; i < msg.taskList.size(); i++)
    {
        auto& t = msg.taskList[i];
        std::cout << t.id << " ";
        std::cout << t.deadline << " ";
        std::cout << t.pickUp << " ";
        std::cout << t.delivery << std::endl;
    }
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
    ros::Subscriber subReq = nh.subscribe("/robot3/requests", 10, callback_RE);
    ros::Subscriber subRob = nh.subscribe("server_robot_data", 10, callback_RD);
    ros::Subscriber subTaskList = nh.subscribe("/robot3/task_list", 10, callback_TL);
    ros::spin();

    return 0;
}