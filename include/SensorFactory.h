#ifndef SENSORFACTORY_H
#define SENSORFACTORY_H
 
#include <memory>
#include <ISensor.h>
 
class SensorFactory 
{ 
public:
    SensorFactory();
    ~SensorFactory();

    enum class SensorType
    {
        GAZEBO_IMU, 
        GAZEBO_GPS, 
        NUM_TYPES
    }; 

    static std::shared_ptr<ISensor> create(const std::string& aSensorType, const std::string& aSensorName); 
    static SensorType stringToEnum(const std::string& aSensorType, const std::string& aSensorName);
private: 
   
};
#endif //SENSORFACTORY_H