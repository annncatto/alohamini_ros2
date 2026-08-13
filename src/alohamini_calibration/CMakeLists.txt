cmake_minimum_required(VERSION 3.8)
project(alohamini_calibration)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config provenance
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

