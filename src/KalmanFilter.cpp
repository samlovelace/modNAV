
#include "KalmanFilter.h"
#include "plog/Log.h"

KalmanFilter::KalmanFilter(std::shared_ptr<IRobot> aRobot) : mRobot(aRobot), 
                        mPrevStateEstimate(Eigen::VectorXd::Zero(aRobot->stateSize())), 
                        mPrevCovarianceMatrix(Eigen::MatrixXd::Zero(aRobot->stateSize(), aRobot->stateSize()))
{

}

KalmanFilter::~KalmanFilter()
{

}

void KalmanFilter::predict()
{
    Eigen::MatrixXd F = mRobot->stateTransitionMatrix();
    Eigen::MatrixXd B = mRobot->controlInputMatrix();
    Eigen::MatrixXd Q = mRobot->processNoiseCovariance();

    // Predict state
    if (B.cols() == mRobot->getPredictionMeasurement().size()) {
        mStatePrediction = F * mPrevStateEstimate + B * mRobot->getPredictionMeasurement();
    } else {
        mStatePrediction = F * mPrevStateEstimate;  // No control input
    }

    LOGW << "mStatePred: " << mStatePrediction; 

    // Predict covariance
    Eigen::MatrixXd F_T = F.transpose();
    mCovariancePrediction = F * mPrevCovarianceMatrix * F_T + Q;
}

void KalmanFilter::update(Eigen::VectorXd z, Eigen::MatrixXd R)
{
    Eigen::MatrixXd H = mRobot->measurementMatrix();

    // Validate dimensions
    if (z.size() != H.rows()) {
        throw std::runtime_error("Measurement size mismatch.");
    }

    // z.resize(6, 1);
    // mStatePrediction.resize(6, 1);

    // Measurement residual
    Eigen::VectorXd predictedMeasurement = H*mStatePrediction; 
    Eigen::VectorXd y = z - predictedMeasurement;

    // Innovation covariance
    Eigen::MatrixXd S = H * mCovariancePrediction * H.transpose() + R;

    // Kalman gain
    mKalmanGain = mCovariancePrediction * H.transpose() * S.llt().solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));

    // Update state
    mStateEstimate = mStatePrediction + mKalmanGain * y;

    // Update covariance
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(mKalmanGain.rows(), mKalmanGain.rows());
    mCovarianceEstimate = (I - mKalmanGain * H) * mCovariancePrediction;

    // Prepare for next cycle
    mPrevStateEstimate = mStateEstimate;
    mPrevCovarianceMatrix = mCovarianceEstimate;
}
