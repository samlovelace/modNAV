#ifndef SENSORFUSION_H
#define SENSORFUSION_H

#include "ConfigManager.h"
#include "KalmanFilter.h"
#include <memory>
#include <map> 

class SensorFusion 
{ 
public:
    SensorFusion(ConfigManager::Config& aConfig); 
    ~SensorFusion();

    void run(); 

private:
    std::unique_ptr<KalmanFilter> mKalmanFilter; 
    ConfigManager::Config& mConfig; 

    void doSensorFusion(); 
};
#endif //SENSORFUSION_H