#ifndef SIMPLEROBOT_H
#define SIMPLEROBOT_H

#include <eigen3/Eigen/Dense>
#include "IRobot.h"

class SimpleRobot : public IRobot
{
public:
    static constexpr int STATE_SIZE = 6; // [x, y, theta, vx, vy, omega]
    static constexpr int CONTROL_SIZE = 3; // [ax, ay, alpha]

    SimpleRobot();
    ~SimpleRobot();

    Eigen::MatrixXd stateTransitionMatrix() override; 
    Eigen::MatrixXd measurementMatrix() override; 
    Eigen::MatrixXd controlInputMatrix() override; 
    Eigen::MatrixXd processNoiseCovariance() override;  
    
    Eigen::VectorXd getPredictionMeasurement() override;
    int stateSize() override {return 6;}
    
    void  SetPredictionMeasurement(const Eigen::VectorXd& aMeasurement) override;

private:
    const double dt = 0.01; // Time step
    Eigen::VectorXd mPredMeasurement; 
};

#endif
