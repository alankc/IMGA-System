#include "communicator.hpp"
#include <iostream>

Communicator::Communicator(/* args */)
{
}

Communicator::~Communicator()
{
    thrRequest->detach();
    thrTask->detach();
    delete (thrRequest);
    delete (thrTask);
}

void Communicator::listnerRequest()
{
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(myResquestTopic, 10, &Communicator::callbackRequest, this);
    while(ros::ok());
}

void Communicator::callbackRequest(const system_client::MsgRequest &msg)
{

}

void Communicator::listnerTask()
{
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(myTaskTopic, 10, &Communicator::callbackTask, this);
    while(ros::ok());
}

void Communicator::callbackTask(const system_client::MsgTaskList &msg)
{
    //adicionar tarefas a lista do controlador
}

void Communicator::sendRobotData()
{
    //Requisitar dados do robô ao controlador
    
    
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<system_client::MsgRobotData>(srvRobotDataTopic, 10);

    system_client::MsgRobotData msg;
    pub.publish(msg);

    ros::spinOnce();
}

void Communicator::run()
{
    thrRequest = new std::thread(&Communicator::listnerRequest, this);
    thrTask = new std::thread(&Communicator::listnerTask, this);
}