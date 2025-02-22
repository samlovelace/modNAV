#ifndef SENSORFUSION_H
#define SENSORFUSION_H

#include "ConfigManager.h"
#include "KalmanFilter.h"
#include <memory>
#include <map> 

class SensorFusion 
{ 
public:
    SensorFusion(ConfigManager::Config aConfig); 
    ~SensorFusion();

    void run(); 

private:
    std::shared_ptr<IRobot> mRobot;  
    std::unique_ptr<KalmanFilter> mKalmanFilter; 
    std::map<std::string, std::shared_ptr<ISensor>> mSensors; 

    ConfigManager::Config mConfig; 

    void doSensorFusion(); 
};
#endif //SENSORFUSION_H