
#include "SensorFusion.h"
#include <chrono>
#include <thread> 
#include "plog/Log.h"

SensorFusion::SensorFusion(ConfigManager::Config& aConfig) : mConfig(aConfig), 
            mKalmanFilter(std::make_unique<KalmanFilter>(aConfig.mRobot))
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
        doSensorFusion();   

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
    if(!mConfig.mSensors.empty() && nullptr != mConfig.mRobot)
    {
        if(mConfig.mSensors.find(ISensor::SensorType::IMU) != mConfig.mSensors.end())
        {
            auto sensor = mConfig.mSensors.at(ISensor::SensorType::IMU); 
            LOGW << "GETTING IMU MEASUREMENT"; 
            auto measurement = sensor->getMeasurement();
            LOGW << "measurement: " << measurement; 
            
            LOGW << "SETTING PRED MEASUREMENT"; 
            mConfig.mRobot->SetPredictionMeasurement(measurement); 
            LOGW << "DONE SETTING PRED MEASUREMENT"; 
        }

        LOGW << "DOING KALMAN PREDICTION"; 
        // kalman filter prediction step
        mKalmanFilter->predict();

        // kalman filter update for each sensor
        for(const auto& [type, sensor] : mConfig.mSensors)
        {
            if(ISensor::SensorType::IMU == type)
            {
                continue; 
            }

            auto measurement = sensor->getMeasurement(); 
            auto covariance = sensor->getMeasurementCovariance(); 
            mKalmanFilter->update(measurement, covariance); 
        }
    }
    else{
        LOGE << "Sensor and Robot do not exist..."; 
        return; 
    }
}