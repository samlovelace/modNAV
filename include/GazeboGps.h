#ifndef GAZEBOGPS_H
#define GAZEBOGPS_H

#include "ISensor.h"
 
class GazeboGps : public ISensor
{ 
public:
    GazeboGps();
    ~GazeboGps();

    Eigen::VectorXd getMeasurement() override; 
    Eigen::MatrixXd getMeasurementCovariance() override; 
    SensorType getType() const override; 

private:
   
};
#endif //GAZEBOIMU_H