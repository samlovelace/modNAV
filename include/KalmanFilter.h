#ifndef KALMANFILTER_H
#define KALMANFILTER_H
 
#include "IRobot.h"
#include <memory>
 
class KalmanFilter 
{ 
public:
    KalmanFilter();
    ~KalmanFilter();

    void predict(); 
    void update(Eigen::VectorXd aMeasurementVector, Eigen::MatrixXd aMeasurementCovarianceMatrix); 

private:
    std::shared_ptr<IRobot> mRobot;
    
    Eigen::VectorXd mPrevStateEstimate; 
    Eigen::VectorXd mStatePrediction; 
    Eigen::VectorXd mStateEstimate; 
    Eigen::MatrixXd mPrevCovarianceMatrix;
    Eigen::MatrixXd mCovariancePrediction; 
    Eigen::MatrixXd mCovarianceEstimate; 

    Eigen::MatrixXd mKalmanGain; 
   
};
#endif //KALMANFILTER_H