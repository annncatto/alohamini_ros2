cmake_minimum_required(VERSION 3.8)
project(alohamini_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes provenance rviz scripts srdf urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
