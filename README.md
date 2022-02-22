# GA-System

The technological advances have led more consumers to adhere to e-commerce shopping. Consequently, the need for efficiency has increased in warehouses. The new technologies also had allowed the use of commercial mobile robots in several areas, including logistics, once that robots are less susceptible to failures, which can increase warehouse efficiency. Besides that, its operational cost does not increase with time, as staff salaries. However, to robots being efficient, it is necessary optimization in the management of task allocation.
  
This process can use several techniques, methods as auction and Genetic Algorithm (GA) are among the most used. However, the approaches already developed do not meet restrictions related to deadline, energy consumption and payload capacities simultaneously. Therefore, this research has developed an architecture that uses the Island Model GA, in multi-robot task allocation, that satisfies the restrictions mentioned above. 

The developed system uses packages from ROS to navigation and communication. Thus any robot set that utilizes a version of this operating system can use the developed architecture.
  
The system evaluation has shown that it allocate more tasks than the standard GA when using the same heuristic. Also, the developed approach finds solutions where the estimated energy consumption is smaller than the expected and meets restrictions of payload, battery and deadline. It has shown also that the system is functional to coordinate a group of robots in a simulated environment. 
  
Finally, the results show that a medium scale real environment can use the system. To larger systems, the approach can find optimized solutions, but the time to execute the allocations process grows exponentially depending on the task set size.

[Portuguese version](https://github.com/alankc/GA-System/blob/master/README_PT_BR.md)
  
## Simulação

To perform the simulation, an environment of approximately 50mx30m was built using the shelves of the small-warehouse package from Amazon: https://github.com/aws-robotics/aws-robomaker-small-warehouse-world 

The model of the robots used in the simulation was adapted from the repository: https://github.com/RafBerkvens/ua_ros_p3dx 

To run the simulation, the user needs to clone this repository in his ROS workspace. Then, compile the system and install and configure the with the system tables. With the requirements met, just run the following commands in the order they appear:

### To run the environment
roslaunch p3dx_simulation sim_alan_warehouse.launch


### To run robots' internal system
roslaunch p3dx_simulation sim_system_client.launch

### To run the coordinator
rosrun system_server system_server_node

## Internal system of the robots
In the following link are the files of the internal system of the robots:
### [system_client](https://github.com/alankc/GA-System/tree/master/system_client)

## Task coordination and allocation system
In the following link are the task allocation and coordination system files:
### [system_server](https://github.com/alankc/GA-System/tree/master/system_server)

## Database
The database uses MySQL and was modeled using MySQL Workbench.
In the following link are the database files:
### [database](https://github.com/alankc/GA-System/tree/master/database)

## Results
Two types of experiments were carried out: one to evaluate the task allocator independently and another to evaluate the coordination system and the functioning of the system as a whole in simulation.

In the following link are the data extracted from the experiments:
### [results](https://github.com/alankc/GA-System/tree/master/results)
