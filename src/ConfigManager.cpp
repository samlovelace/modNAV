
#include "ConfigManager.h"
#include <yaml-cpp/yaml.h>
#include "plog/Log.h"
#include "RobotFactory.h"
#include "SensorFactory.h"


bool ConfigManager::parseConfig(const std::string& aFilename)
{
    LOGD << "Attempting to read config file at " << aFilename; 
    YAML::Node config = YAML::LoadFile(aFilename);
    
    if(!config["robot"])
    {
        LOGE << "Error reading YAML config file"; 
        return false; 
    }

    LOGD << "Successfully read config file!";
    
    std::stringstream s; 
    s << "Robot Configuration: \n";
    s << YAML::Dump(config); 
    LOGD << s.str(); 

    mConfig.mRobot = RobotFactory::create(config["robot"]["type"].as<std::string>());

    if(nullptr == mConfig.mRobot)
    {
        LOGE << "A valid robot was not configured. Use 'robot:' as the top level node."; 
        return false; 
    }

    mConfig.mSettings.mUpdateRate = config["robot"]["nav_update_rate"].as<int>(); 


    return true; 
}