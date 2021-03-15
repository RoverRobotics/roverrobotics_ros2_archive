cmake_minimum_required(VERSION 2.8.3)
project(roverrobotics_description)

find_package(ament_cmake REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY meshes/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
    FILES_MATCHING PATTERN "*"
)

install(DIRECTORY urdf/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
    FILES_MATCHING PATTERN "*"
)
