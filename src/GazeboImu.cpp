
#include "GazeboImu.h"
#include "plog/Log.h"

GazeboImu::GazeboImu()
{
    LOGD << "GazeboIMU Constructed!"; 

}

GazeboImu::~GazeboImu()
{

}

Eigen::VectorXd GazeboImu::getMeasurement()
{
    
}

Eigen::MatrixXd GazeboImu::getMeasurementCovariance()
{

}
std::string GazeboImu::getType() const
{

}