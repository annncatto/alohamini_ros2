cmake_minimum_required(VERSION 3.8)
project(alohamini_calibration)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config provenance
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/calibrate_arms scripts/calibrate_lift_axis scripts/sync_arm_mapping
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_calibrate_arms test/test_calibrate_arms.py)
  ament_add_pytest_test(test_lift_axis_calibration test/test_lift_axis_calibration.py)
  ament_add_pytest_test(test_sync_arm_mapping test/test_sync_arm_mapping.py)
endif()

ament_package()
