#ifndef GAZEBOIMU_H
#define GAZEBOIMU_H

#include "ISensor.h"
#include <mutex> 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
 
class GazeboImu : public ISensor, rclcpp::Node
{ 
public:
    GazeboImu();
    ~GazeboImu();

    Eigen::VectorXd getMeasurement();
    Eigen::MatrixXd getMeasurementCovariance();
    std::string getType() const;

private:

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr mSubscriber; 
    void sensorCallback(sensor_msgs::msg::Imu::SharedPtr aMsg); 
     
    std::thread mSpinThread;
    Eigen::VectorXd mLatestMeasurement; 
    std::mutex mLatestMeasurementMutex;
   
};
#endif //GAZEBOIMU_H