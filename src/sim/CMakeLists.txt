cmake_minimum_required(VERSION 3.5)
project(sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds models
  DESTINATION share/${PROJECT_NAME}
)


install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
