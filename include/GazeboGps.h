#ifndef GAZEBOGPS_H
#define GAZEBOGPS_H

#include "ISensor.h"
 
class GazeboGps : public ISensor
{ 
public:
    GazeboGps();
    ~GazeboGps();

    Eigen::VectorXd getMeasurement();
    Eigen::MatrixXd getMeasurementCovariance();
    std::string getType() const;

private:
   
};
#endif //GAZEBOIMU_H