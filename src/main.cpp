
#include "SensorFusion.h"
#include "SimpleRobot.h"
#include "Logger.h"
#include "ConfigManager.h"

int main()
{  
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

}
