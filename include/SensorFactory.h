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
        IMU, 
        GPS, 
        NUM_TYPES
    }; 

    static std::shared_ptr<ISensor> create(SensorType aType); 

private:
   
};
#endif //SENSORFACTORY_H