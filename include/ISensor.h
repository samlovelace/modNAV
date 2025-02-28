#ifndef ISENSOR_H
#define ISENSOR_H
 
#include <string> 
#include <eigen3/Eigen/Dense>
 
class ISensor 
{ 
public:
    virtual ~ISensor() = default; 

    enum class SensorType
    {
        IMU, 
        GPS, 
        SONAR, 
        LIDAR, 
        CAMERA, 
        NUM_TYPES
    };

    virtual Eigen::VectorXd getMeasurement() = 0; 
    virtual Eigen::MatrixXd getMeasurementCovariance() = 0; 
    virtual SensorType getType() const = 0; 


protected:

   
};
#endif //ISENSOR_H