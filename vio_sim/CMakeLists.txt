cmake_minimum_required(VERSION 3.8)
project(vio_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  models
  worlds
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
