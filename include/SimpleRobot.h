#ifndef SIMPLEROBOT_H
#define SIMPLEROBOT_H
 
#include "IRobot.h" 

class SimpleRobot : public IRobot
{ 
public:
    SimpleRobot();
    ~SimpleRobot();

    Eigen::MatrixXd stateTransitionMatrix();
    Eigen::MatrixXd measurementMatrix(); 
    Eigen::MatrixXd controlInputMatrix(); 
    Eigen::MatrixXd processNoiseCovariance(); 
    Eigen::MatrixXd controlInput(); 

private:
   
};
#endif //SIMPLEROBOT_H