#ifndef IROBOT_H
#define IROBOT_H
 
#include <eigen3/Eigen/Dense>
 
class IRobot 
{ 
public:
    virtual ~IRobot() = default; 

    virtual Eigen::MatrixXd stateTransitionMatrix() = 0; 
    virtual Eigen::MatrixXd measurementMatrix() = 0; 
    virtual Eigen::MatrixXd controlInputMatrix() = 0; 
    virtual Eigen::MatrixXd processNoiseCovariance() = 0; 
     
    virtual Eigen::VectorXd getPredictionMeasurement() = 0; 
    virtual int stateSize() = 0; 

    virtual void SetPredictionMeasurement(const Eigen::VectorXd& aControlInput) = 0;

protected: 
   
};
#endif //IROBOT_H