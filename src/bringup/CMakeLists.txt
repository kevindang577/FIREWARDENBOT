cmake_minimum_required(VERSION 3.5)
project(bringup)

find_package(ament_cmake REQUIRED)

# install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# install config files (yaml, rviz)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
