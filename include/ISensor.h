#ifndef ISENSOR_H
#define ISENSOR_H
 
#include <string> 
#include <eigen3/Eigen/Dense>
 
class ISensor 
{ 
public:
    virtual ~ISensor() = default; 

    virtual Eigen::VectorXd getMeasurement() = 0; 
    virtual Eigen::MatrixXd getMeasurementCovariance() = 0; 
    virtual std::string getType() const = 0; 

protected:

   
};
#endif //ISENSOR_H