#include "generalDao.hpp"
#include <iostream>

GeneralDao::GeneralDao(){}

GeneralDao::GeneralDao(std::string host, std::string user, std::string pass, std::string database)
{
    this->host = host;
    this->user = user;
    this->pass = pass;
    this->database = database;
}

GeneralDao::~GeneralDao()
{
}

bool GeneralDao::executeQuery(std::string statement, std::unique_ptr<sql::ResultSet> &res)
{
    bool sucess = true;
    try
    {
        sql::Driver *driver = get_driver_instance();

        std::unique_ptr<sql::Connection> con(driver->connect(host, user, pass));
        con->setSchema(database);

        std::unique_ptr<sql::PreparedStatement> pstmt;

        pstmt.reset(con->prepareStatement(statement));
        res.reset(pstmt->executeQuery());
    }
    catch (sql::SQLException &e)
    {
        /*
      MySQL Connector/C++ throws three different exceptions:

      - sql::MethodNotImplementedException (derived from sql::SQLException)
      - sql::InvalidArgumentException (derived from sql::SQLException)
      - sql::SQLException (derived from std::runtime_error)
    */
        std::cout << "# ERR: SQLException in " << __FILE__;
        std::cout << "(" << __FUNCTION__ << ") on line " << __LINE__ << std::endl;
        /* what() (derived from std::runtime_error) fetches error message */
        std::cout << "# ERR: " << e.what();
        std::cout << " (MySQL error code: " << e.getErrorCode();
        std::cout << ", SQLState: " << e.getSQLState() << " )" << std::endl;

        sucess = false;
    }

    return sucess;
}

bool GeneralDao::executeUpdate(std::string statement)
{
    bool sucess = true;
    try
    {
        sql::Driver *driver = get_driver_instance();

        std::unique_ptr<sql::Connection> con(driver->connect(host, user, pass));
        con->setSchema(database);

        std::unique_ptr<sql::PreparedStatement> pstmt;
        pstmt.reset(con->prepareStatement(statement));
        pstmt->executeUpdate();
    }
    catch (sql::SQLException &e)
    {
        /*
      MySQL Connector/C++ throws three different exceptions:

      - sql::MethodNotImplementedException (derived from sql::SQLException)
      - sql::InvalidArgumentException (derived from sql::SQLException)
      - sql::SQLException (derived from std::runtime_error)
    */
        std::cout << "# ERR: SQLException in " << __FILE__;
        std::cout << "(" << __FUNCTION__ << ") on line " << __LINE__ << std::endl;
        /* what() (derived from std::runtime_error) fetches error message */
        std::cout << "# ERR: " << e.what();
        std::cout << " (MySQL error code: " << e.getErrorCode();
        std::cout << ", SQLState: " << e.getSQLState() << " )" << std::endl;

        sucess = false;
    }

    return sucess;
}

bool GeneralDao::executeUpdate(std::vector<std::string> &statement)
{
    bool sucess = true;
    std::unique_ptr<sql::Connection> con;

    try
    {
        sql::Driver *driver = get_driver_instance();


        con.reset(driver->connect(host, user, pass));
        con->setSchema(database);
        con->setAutoCommit(false);

        std::unique_ptr<sql::PreparedStatement> pstmt;

        for (auto s : statement)
        {
            pstmt.reset(con->prepareStatement(s));
            pstmt->executeUpdate();
        }

        con->commit();
    }
    catch (sql::SQLException &e)
    {
        con->rollback();
        /*
      MySQL Connector/C++ throws three different exceptions:

      - sql::MethodNotImplementedException (derived from sql::SQLException)
      - sql::InvalidArgumentException (derived from sql::SQLException)
      - sql::SQLException (derived from std::runtime_error)
    */
        std::cout << "# ERR: SQLException in " << __FILE__;
        std::cout << "(" << __FUNCTION__ << ") on line " << __LINE__ << std::endl;
        /* what() (derived from std::runtime_error) fetches error message */
        std::cout << "# ERR: " << e.what();
        std::cout << " (MySQL error code: " << e.getErrorCode();
        std::cout << ", SQLState: " << e.getSQLState() << " )" << std::endl;

        sucess = false;
    }

    return sucess;
}