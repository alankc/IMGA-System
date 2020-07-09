#ifndef COMMUNICATOR_HPP
#define COMMUNICATOR_HPP

#include <ros/ros.h>
#include <thread>

class Communicator
{
private:
   std::thread* thrRequest; 
   std::thread* thrTask;

    void listnerRequest();
    void callbackRequest();
    void listnerTask();
    void callbackTask();
    void sendRobotData();

public:
    Communicator(/* args */);
    ~Communicator();
    void run();
};

#endif