#ifndef GAZEBOGPS_H
#define GAZEBOGPS_H

#include "ISensor.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <thread>

class GazeboGps : public ISensor, public rclcpp::Node
{ 
public:
    GazeboGps();
    ~GazeboGps();

    Eigen::VectorXd getMeasurement() override; 
    Eigen::MatrixXd getMeasurementCovariance() override; 
    SensorType getType() const override; 

private:
    void sensorCallback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr aMsg); 

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr mSubscriber; 

    std::thread mSpinThread;
    std::mutex mMutex;

    Eigen::VectorXd mState;       // [x, y, theta, vx, vy, omega]
    Eigen::MatrixXd mCovariance;  // Measurement covariance

    Eigen::VectorXd mLastState;   // Previous state for velocity computation
    rclcpp::Time mLastTimestamp;  // Last received timestamp
};

#endif // GAZEBOGPS_H
