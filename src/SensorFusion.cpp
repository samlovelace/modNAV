
#include "SensorFusion.h"
#include <chrono>
#include <thread> 
#include "plog/Log.h"

SensorFusion::SensorFusion(ConfigManager::Config aConfig) : mConfig(aConfig), mRobot(aConfig.mRobot), mSensors(aConfig.mSensors)
{

}

SensorFusion::~SensorFusion()
{

}

void SensorFusion::run()
{
    bool done = false; 

    // frequency to perform state estimation at 
    std::chrono::duration<double> loop_duration(1.0/mConfig.mSettings.mUpdateRate); 
    
    while(!done)
    {
        auto start = std::chrono::steady_clock::now(); 
 
        LOGD << "doing SensorFusion"; 
        //doSensorFusion();   

        auto end = std::chrono::steady_clock::now(); 
        auto elapsed = end - start; 
        if(elapsed < loop_duration)
        {
            std::this_thread::sleep_for(loop_duration - elapsed); 
        }else{
            LOGW << "main loop went long :("; 
        }
    }

}

void SensorFusion::doSensorFusion()
{
    mKalmanFilter->predict(); 
    
    for(const auto& [type, sensor] : mSensors)
    {
        auto measurement = sensor->getMeasurement();
        auto covariance = sensor->getMeasurementCovariance();  
        mKalmanFilter->update(measurement, covariance); 
    }
}