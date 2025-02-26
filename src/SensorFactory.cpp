
#include "SensorFactory.h"
#include "plog/Log.h"
#include "GazeboImu.h"
#include "GazeboGps.h"

std::shared_ptr<ISensor> SensorFactory::create(const std::string& aSensorType, const std::string& aSensorName)
{
    SensorType typeToCreate = stringToEnum(aSensorType, aSensorName); 

    if(SensorType::GAZEBO_IMU == typeToCreate)
    {
        return std::make_shared<GazeboImu>(); 
    }
    else if (SensorType::GAZEBO_GPS == typeToCreate)
    {
        return std::make_shared<GazeboGps>(); 
    }
    else{
        LOGE << "Unsupported sensor: " << aSensorName << " " << aSensorType; 
        return nullptr; 
    }

    
}

SensorFactory::SensorType SensorFactory::stringToEnum(const std::string& aSensorType, const std::string& aSensorName)
{
    SensorFactory::SensorType enumToReturn; 

    if("gazebo" == aSensorName || "Gazebo" == aSensorName)
    {
        if("imu" == aSensorType || "IMU" == aSensorType)
        {
            enumToReturn = SensorType::GAZEBO_IMU; 
        }
        else if ("gps" == aSensorType || "GPS" == aSensorType)
        {
            enumToReturn = SensorType::GAZEBO_GPS; 
        } 
        else{
            LOGE << "Unsupported gazebo sensor with name: " << aSensorName.c_str(); 
            return SensorFactory::SensorType::NUM_TYPES; 
        }
    }
    else 
    {
        LOGE << "Unsupported sensor type: " << aSensorType.c_str(); 
        return SensorFactory::SensorType::NUM_TYPES; 
    }

    return enumToReturn; 
}