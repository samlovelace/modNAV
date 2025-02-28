#include "SimpleRobot.h"
#include "plog/Log.h"

SimpleRobot::SimpleRobot() : mPredMeasurement(Eigen::VectorXd::Zero(CONTROL_SIZE))
{
}

SimpleRobot::~SimpleRobot()
{
}

// 🔹 State Transition Matrix (A)
Eigen::MatrixXd SimpleRobot::stateTransitionMatrix()
{
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);

    // Update position based on velocity
    A(0, 3) = dt; // x' = x + vx * dt
    A(1, 4) = dt; // y' = y + vy * dt
    A(2, 5) = dt; // theta' = theta + omega * dt

    return A;
}

// 🔹 Measurement Matrix (H) - Relates state to sensor readings
Eigen::MatrixXd SimpleRobot::measurementMatrix()
{
    Eigen::MatrixXd H(4, STATE_SIZE); // Assuming sensors measure x, y, theta, omega
    H.setZero();

    H(0, 0) = 1; // Measuring x position
    H(1, 1) = 1; // Measuring y position
    H(2, 2) = 1; // Measuring theta
    H(3, 5) = 1; // Measuring omega

    return H;
}

// 🔹 Control Input Matrix (B)
Eigen::MatrixXd SimpleRobot::controlInputMatrix()
{
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(STATE_SIZE, CONTROL_SIZE);

    // Acceleration affects velocity
    B(3, 0) = dt; // vx' += ax * dt
    B(4, 1) = dt; // vy' += ay * dt
    B(5, 2) = dt; // omega' += alpha * dt

    return B;
}

// 🔹 Process Noise Covariance (Q)
Eigen::MatrixXd SimpleRobot::processNoiseCovariance()
{
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 0.01;

    Q(0, 0) = 0.1; // x uncertainty
    Q(1, 1) = 0.1; // y uncertainty
    Q(2, 2) = 0.01; // theta uncertainty
    Q(3, 3) = 0.1; // vx uncertainty
    Q(4, 4) = 0.1; // vy uncertainty
    Q(5, 5) = 0.01; // omega uncertainty

    return Q;
}

// 🔹 Control Input Vector (u) - IMU readings as input
void SimpleRobot::SetPredictionMeasurement(const Eigen::VectorXd& aMeasurement)
{
    mPredMeasurement << aMeasurement(0), aMeasurement(1), aMeasurement(5);
    LOGW << "Stil in fcn after settting mpredmeasurement"; 
}

Eigen::VectorXd SimpleRobot::getPredictionMeasurement()
{
    return mPredMeasurement; 
}
