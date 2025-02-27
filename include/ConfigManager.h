#ifndef CONFIGMANAGER_H
#define CONFIGMANAGER_H
 
#include <memory> 
#include <string> 
#include "IRobot.h"
#include "ISensor.h"
#include <map> 
 
class ConfigManager 
{ 
public:
    static ConfigManager* getInstance() {
        static ConfigManager* instance = new ConfigManager(); 
        return instance; 
    }

    struct Settings
    {
        int mUpdateRate;
    };

    struct Config
    {
        Settings mSettings; 
        std::shared_ptr<IRobot> mRobot;
        std::map<std::string, std::shared_ptr<ISensor>> mSensors; 
    };


    bool parseConfig(const std::string& aFilename); 
    Config& getConfig() {return mConfig; }

private:
    ConfigManager() {}
    ~ConfigManager() {}

    Config mConfig; 
   
};
#endif //CONFIGMANAGER_H