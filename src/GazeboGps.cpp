
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
std::string GazeboGps::getType() const
{

}