#ifndef GAZEBOIMU_H
#define GAZEBOIMU_H

#include "ISensor.h"
 
class GazeboImu : public ISensor
{ 
public:
    GazeboImu();
    ~GazeboImu();

    Eigen::VectorXd getMeasurement();
    Eigen::MatrixXd getMeasurementCovariance();
    std::string getType() const;

private:
   
};
#endif //GAZEBOIMU_H