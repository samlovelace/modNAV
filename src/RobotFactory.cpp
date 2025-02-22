
#include "RobotFactory.h"
#include "SimpleRobot.h"
#include "plog/Log.h"

std::shared_ptr<IRobot> RobotFactory::create(const std::string& aRobotType)
{
    RobotFactory::RobotType type = RobotFactory::stringToEnum(aRobotType); 

    if(RobotType::SIMPLE == type)
    {
        return std::make_shared<SimpleRobot>();  
    }
    else{
        LOGE << "Failed to create robot of type: " << aRobotType;
        return nullptr; 
    }
}

RobotFactory::RobotType RobotFactory::stringToEnum(const std::string& aRobotType)
{
    RobotFactory::RobotType enumToReturn; 

    if("SimpleRobot" == aRobotType || "simple" == aRobotType)
    {
        enumToReturn = RobotFactory::RobotType::SIMPLE; 
    }
    else 
    {
        LOGE << "Unsupported robot type: " << aRobotType.c_str(); 
        return RobotFactory::RobotType::NUM_TYPES; 
    }

    return enumToReturn; 
}

