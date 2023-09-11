cmake_minimum_required(VERSION 3.8)
project(learning_based_vehicle_calibration)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(
  PROGRAMS
  scripts/data_collection.py
  scripts/data_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

# install
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)