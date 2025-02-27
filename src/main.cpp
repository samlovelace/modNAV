
#include "SensorFusion.h"
#include "SimpleRobot.h"
#include "Logger.h"
#include "ConfigManager.h"
#include <rclcpp/rclcpp.hpp>

int main()
{  
  std::signal(SIGINT, signalHandler); 
  
  // eventually planning to have a pose publishing interface using ros2 so this should be fine here
  // would like to make the core agnostic to messaging type
  rclcpp::init(0, nullptr); 

  createLogger();

  std::string configFilename = "./src/modNAV/config.yaml"; 
  auto configManager = ConfigManager::getInstance(); 
  
  if(!configManager->parseConfig(configFilename))
  {
    LOGE << "Failed to parse config file :("; 
    return 0; 
  }
  
  SensorFusion sf(configManager->getConfig());
  sf.run(); 

  rclcpp::shutdown(); 
}
