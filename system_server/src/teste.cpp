#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <stdexcept>
/* uncomment for applications that use vectors */
/*#include <vector>
g++ -o test_mysql -I/usr/local/include -I/usr/local/include/cppconn test_mysql.cpp -lmysqlcppconn
https://answers.ros.org/question/218978/using-the-mysql-database-in-ros-jade-project/
*/

#include "controller/generalController.hpp"

#define EXAMPLE_HOST "localhost"
#define EXAMPLE_USER "root"
#define EXAMPLE_PASS "281094"
#define EXAMPLE_DB "ServerDB"

int main(int argc, const char **argv)
{
    std::string host = "localhost";
    std::string user = "root";
    std::string pass = "281094";
    std::string db = "ServerDB";
    GeneralController gc(host, user, pass, db);

    gc.callScheduler();
    
    
    return EXIT_SUCCESS;
}
