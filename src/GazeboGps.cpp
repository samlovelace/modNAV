#include "GazeboGps.h"
#include "plog/Log.h"
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>

GazeboGps::GazeboGps() : Node("GPS")
{
    LOGD << "GazeboGPS Constructed!";

    // Subscribe to the correct topic
    mSubscriber = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/robot/pose", 10,
        std::bind(&GazeboGps::sensorCallback, this, std::placeholders::_1));

    // Initialize state vector (6 elements: x, y, theta, vx, vy, omega)
    mState.setZero(6);

    // Start ROS 2 spinning in a separate thread
    mSpinThread = std::thread([this]() {
        rclcpp::spin(this->get_node_base_interface());
    });
}

GazeboGps::~GazeboGps()
{
    // Ensure the spin thread is safely stopped
    rclcpp::shutdown();
    if (mSpinThread.joinable()) {
        mSpinThread.join();
    }
}

Eigen::VectorXd GazeboGps::getMeasurement()
{
    std::lock_guard<std::mutex> lock(mMutex);
    return mState; // Return the latest state estimate
}

Eigen::MatrixXd GazeboGps::getMeasurementCovariance()
{
    std::lock_guard<std::mutex> lock(mMutex);
    return mCovariance; // Return the latest covariance matrix
}

ISensor::SensorType GazeboGps::getType() const
{
    return ISensor::SensorType::GPS;
}

void GazeboGps::sensorCallback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr aMsg)
{
    std::lock_guard<std::mutex> lock(mMutex);

    // Extract pose data
    double x = aMsg->pose.pose.position.x;
    double y = aMsg->pose.pose.position.y;
    double qw = aMsg->pose.pose.orientation.w;
    double qz = aMsg->pose.pose.orientation.z;

    // Convert quaternion to yaw angle (theta)
    double theta = 2.0 * std::atan2(qz, qw);

    // Ensure timestamp uses ROS simulation time
    rclcpp::Time current_time(aMsg->header.stamp, RCL_ROS_TIME);

    // Compute velocities using finite difference method
    double vx = 0.0, vy = 0.0, omega = 0.0;

    // Only compute velocity if we have a valid previous timestamp
    if (mLastTimestamp.nanoseconds() > 0)
    {
        rclcpp::Duration dt = current_time - mLastTimestamp;
        double dt_sec = dt.seconds();

        if (dt_sec > 0)
        {
            vx = (x - mLastState(0)) / dt_sec;
            vy = (y - mLastState(1)) / dt_sec;
            omega = (theta - mLastState(2)) / dt_sec;
        }
    }

    // Update state vector [x, y, theta, vx, vy, omega]
    mState << x, y, theta, vx, vy, omega;
    mLastState = mState; // Store for next velocity calculation
    mLastTimestamp = current_time; // Update last timestamp

    // Extract 6x6 covariance matrix from pose message
    Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(6, 6);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            cov(i, j) = aMsg->pose.covariance[i * 6 + j]; // Copy pose covariance into state covariance
        }
    }
    mCovariance = cov;

    // LOGD << "Updated GPS Measurement: [" << x << ", " << y << ", " << theta
    //      << ", " << vx << ", " << vy << ", " << omega << "]";
}

