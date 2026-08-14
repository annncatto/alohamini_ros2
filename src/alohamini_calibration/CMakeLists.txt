cmake_minimum_required(VERSION 3.8)
project(alohamini_calibration)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config provenance
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/calibrate_lift_axis
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_lift_axis_calibration test/test_lift_axis_calibration.py)
endif()

ament_package()
