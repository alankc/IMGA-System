#include "settingsDao.hpp"

SettingsDao::SettingsDao()
{
}

SettingsDao::SettingsDao(GeneralDao *gDao)
{
    this->gDao = gDao;
}

SettingsDao::~SettingsDao()
{
}

bool SettingsDao::getSettings(uint32_t id, Settings &settings)
{
    std::unique_ptr<sql::ResultSet> res;
    std::string stmt = "SELECT * FROM settings WHERE id_setup = " + std::to_string(id) + ";";

    bool tst = gDao->executeQuery(stmt, res);

    if (tst)
    {
        if (res->next())
        {
            settings.setIdSetup(id);
            settings.setTaskPoolSize(res->getUInt("task_pool_size"));
            settings.setRobotPoolSize(res->getUInt("robot_pool_size"));
            settings.setTimeInterval(res->getUInt("time_interval"));
            settings.setGaIterations(res->getUInt("ga_iterations"));
            settings.setGaSubIterations(res->getUInt("ga_sub_iterations"));
            settings.setGaPopulation(res->getUInt("ga_population"));
            settings.setGaTimeLimit(res->getUInt("ga_time_limit"));
            settings.setGaNoChangeLimit(res->getUInt("ga_no_change_limit"));
            settings.setGaElitism(res->getDouble("ga_elitism"));
            settings.setGaMutation(res->getDouble("ga_mutation"));
            settings.setGaMigration(res->getDouble("ga_migration"));
        }
        else
            tst = false;
    }

    return tst;
}