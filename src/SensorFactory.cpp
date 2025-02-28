
#include "SensorFactory.h"
#include "plog/Log.h"
#include "GazeboImu.h"
#include "GazeboGps.h"

std::shared_ptr<ISensor> SensorFactory::create(const std::string& aSensorType, const std::string& aSensorName)
{
    SensorImpl typeToCreate = stringToEnum(aSensorType, aSensorName); 

    if(SensorImpl::GAZEBO_IMU == typeToCreate)
    {
        return std::make_shared<GazeboImu>(); 
    }
    else if (SensorImpl::GAZEBO_GPS == typeToCreate)
    {
        return std::make_shared<GazeboGps>(); 
    }
    else{
        LOGE << "Unsupported sensor: " << aSensorName << " " << aSensorType; 
        return nullptr; 
    }

    
}

SensorFactory::SensorImpl SensorFactory::stringToEnum(const std::string& aSensorImpl, const std::string& aSensorName)
{
    SensorFactory::SensorImpl enumToReturn; 

    if("gazebo" == aSensorName || "Gazebo" == aSensorName)
    {
        if("imu" == aSensorImpl || "IMU" == aSensorImpl)
        {
            enumToReturn = SensorImpl::GAZEBO_IMU; 
        }
        else if ("gps" == aSensorImpl || "GPS" == aSensorImpl)
        {
            enumToReturn = SensorImpl::GAZEBO_GPS; 
        } 
        else{
            LOGE << "Unsupported gazebo sensor with name: " << aSensorName.c_str(); 
            return SensorFactory::SensorImpl::NUM_TYPES; 
        }
    }
    else 
    {
        LOGE << "Unsupported sensor type: " << aSensorImpl.c_str(); 
        return SensorFactory::SensorImpl::NUM_TYPES; 
    }

    return enumToReturn; 
}