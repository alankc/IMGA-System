#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <stdexcept>
/* uncomment for applications that use vectors */
/*#include <vector>
g++ -o test_mysql -I/usr/local/include -I/usr/local/include/cppconn test_mysql.cpp -lmysqlcppconn
https://answers.ros.org/question/218978/using-the-mysql-database-in-ros-jade-project/
*/

#include "mysql_connection.h"

#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>

#include "dao/generalDao.hpp"
#include "dao/robotDao.hpp"
#include "controller/generalController.hpp"

#define EXAMPLE_HOST "localhost"
#define EXAMPLE_USER "root"
#define EXAMPLE_PASS "281094"
#define EXAMPLE_DB "ServerDB"

using namespace std;

int main(int argc, const char **argv)
{
    string url(argc >= 2 ? argv[1] : EXAMPLE_HOST);
    const string user(argc >= 3 ? argv[2] : EXAMPLE_USER);
    const string pass(argc >= 4 ? argv[3] : EXAMPLE_PASS);
    const string database(argc >= 5 ? argv[4] : EXAMPLE_DB);

    GeneralDao gDAO(url, user, pass, database);

    GeneralController gc(url, user, pass, database);

    gc.callScheduler();

    /*RobotDao tmpDao(&gDAO);
    
    RobotRequestData r1;
    r1.id = 0;
    r1.currentLocation = 3;
    r1.mediumVelocity = 3.3;
    r1.remainingBattery = 17.96;
    r1.status = "W";

    tmpDao.updateRobotRequest(r1);*/
   
    /*gDAO.executeUpdate(stmt);

    while (results->next())
    {
        std::cout << "Name: " << results->getUInt("id_location")
                  << " Population: " << results->getString("description")
                  << " Is depot: " << results->getBoolean("is_depot")
                  << " x pos: " << results->getUInt("x_pos")
                  << " y pos: " << results->getUInt("y_pos")
                  << " a pos: " << results->getDouble("a_pos")
                  << std::endl;
    }*/


    /*cout << "Connector/C++ tutorial framework..." << endl;
    cout << endl;

    try
    {

        sql::Driver *driver = get_driver_instance();

        std::unique_ptr<sql::Connection> con(driver->connect(url, user, pass));
        con->setSchema(database);

        std::unique_ptr<sql::PreparedStatement> pstmt;
        std::unique_ptr<sql::ResultSet> res;

        pstmt.reset(con->prepareStatement("SELECT * FROM location;"));
        res.reset(pstmt->executeQuery());

        for (;;)
        {
            while (res->next())
            {
                cout << "Name: " << res->getUInt("id_location")
                     << " Population: " << res->getString("description")
                     << " Is depot: " << res->getBoolean("is_depot")
                     << " x pos: " << res->getUInt("x_pos")
                     << " y pos: " << res->getUInt("y_pos")
                     << " a pos: " << res->getDouble("a_pos")
                     << endl;
            }
            if (pstmt->getMoreResults())
            {
                res.reset(pstmt->getResultSet());
                continue;
            }
            break;
        }
    }
    catch (sql::SQLException &e)
    {

        cout << "# ERR: SQLException in " << __FILE__;
        cout << "(" << __FUNCTION__ << ") on line " << __LINE__ << endl;

        cout << "# ERR: " << e.what();
        cout << " (MySQL error code: " << e.getErrorCode();
        cout << ", SQLState: " << e.getSQLState() << " )" << endl;

        return EXIT_FAILURE;
    }

    cout << "Done." << endl;*/
    return EXIT_SUCCESS;
}
