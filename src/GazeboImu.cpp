
#include "GazeboImu.h"
#include "plog/Log.h"
#include <rclcpp/rclcpp.hpp>

GazeboImu::GazeboImu() : Node("modnav")
{
    LOGD << "GazeboIMU Constructed!"; 

    mSubscriber = create_subscription<sensor_msgs::msg::Imu>("/robot/imu", 10, 
                    std::bind(&GazeboImu::sensorCallback, this, std::placeholders::_1)); 

    mSpinThread = std::thread([this]() {
        rclcpp::spin(this->get_node_base_interface());
        }); 

}

GazeboImu::~GazeboImu()
{
    if(mSpinThread.joinable())
    {
        mSpinThread.join(); 
    }
}

Eigen::VectorXd GazeboImu::getMeasurement()
{
    std::lock_guard<std::mutex> lock(mLatestMeasurementMutex); 
    return mLatestMeasurement; 
}

Eigen::MatrixXd GazeboImu::getMeasurementCovariance()
{

}
std::string GazeboImu::getType() const
{
    return "imu"; 
}

void GazeboImu::sensorCallback(sensor_msgs::msg::Imu::SharedPtr aMsg)
{
    Eigen::VectorXd measurement(6); 
    geometry_msgs::msg::Vector3 acc = aMsg->linear_acceleration; 
    geometry_msgs::msg::Vector3 ang_vel = aMsg->angular_velocity; 

    measurement << acc.x, acc.y, acc.z, ang_vel.x, ang_vel.y, ang_vel.z; 

    std::lock_guard<std::mutex> lock(mLatestMeasurementMutex); 
    mLatestMeasurement = measurement; 
}