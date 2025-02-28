
#include "GazeboGps.h"
#include "plog/Log.h"

GazeboGps::GazeboGps()
{
    LOGD << "GazeboGPS Constructed!"; 
}

GazeboGps::~GazeboGps()
{

}

Eigen::VectorXd GazeboGps::getMeasurement()
{
    
}

Eigen::MatrixXd GazeboGps::getMeasurementCovariance()
{

}

ISensor::SensorType GazeboGps::getType() const
{
    return ISensor::SensorType::GPS; 
}