#ifndef COMMUNICATOR_HPP
#define COMMUNICATOR_HPP

#include <ros/ros.h>
#include <thread>
#include <string>
#include <system_client/MsgRequest.h>
#include <system_client/MsgTaskList.h>
#include <system_client/MsgRobotData.h>

class Communicator
{
private:
    std::string srvRequestTopic;
    std::string srvRobotDataTopic;
    std::string myResquestTopic;
    std::string myTaskTopic;

    std::thread *thrRequest;
    std::thread *thrTask;

    void listnerRequest();
    void callbackRequest(const system_client::MsgRequest &msg);
    void listnerTask();
    void callbackTask(const system_client::MsgTaskList &msg);
    void sendRobotData();

public:
    Communicator(/* args */);
    ~Communicator();
    void run();
};

#endif