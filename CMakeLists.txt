cmake_minimum_required(VERSION 3.8)
project(modNAV)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(SOURCES 
  ${CMAKE_CURRENT_LIST_DIR}/src/ConfigManager.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/GazeboImu.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/GazeboGps.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/KalmanFilter.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/RobotFactory.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/SensorFactory.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/SensorFusion.cpp  
  ${CMAKE_CURRENT_LIST_DIR}/src/SimpleRobot.cpp
)

add_executable(modNAV ${SOURCES})

include_directories(${PROJECT_NAME}
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${CMAKE_CURRENT_LIST_DIR}/dependencies/plog/plog-master/include
)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

ament_target_dependencies(
  modNAV
  "rclcpp"
  "Eigen3"
  "yaml-cpp"
)

install(TARGETS modNAV
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
