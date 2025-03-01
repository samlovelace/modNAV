#ifndef KALMANFILTER_H
#define KALMANFILTER_H
 
#include "IRobot.h"
#include <memory>
 
class KalmanFilter 
{ 
public:
    KalmanFilter(std::shared_ptr<IRobot> aRobot);
    ~KalmanFilter();

    void predict(); 
    void update(Eigen::VectorXd aMeasurementVector, Eigen::MatrixXd aMeasurementCovarianceMatrix); 

    Eigen::VectorXd getStateEstimate() {return mStateEstimate; }

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