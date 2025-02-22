#ifndef ROBOTFACTORY_H
#define ROBOTFACTORY_H
 
#include "IRobot.h"
#include <memory>
 
class RobotFactory 
{ 
public:
    RobotFactory();
    ~RobotFactory();

    enum class RobotType
    {
        SIMPLE, 
        NUM_TYPES
    };

    static std::shared_ptr<IRobot> create(const std::string& aRobotType); 
    static RobotFactory::RobotType stringToEnum(const std::string& aRobotType);

private:
   
};
#endif //ROBOTFACTORY_H