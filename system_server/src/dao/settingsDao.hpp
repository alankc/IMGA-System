#ifndef SETTINGS_DAO_HPP
#define SETTINGS_DAO_HPP

#include "generalDao.hpp"
#include "../model/settings.hpp"

class SettingsDao
{
private:
    GeneralDao *gDao;

public:
    SettingsDao();
    SettingsDao(GeneralDao *gDao);
    ~SettingsDao();

    bool getSettings(uint32_t id, Settings &settings);
};

#endif