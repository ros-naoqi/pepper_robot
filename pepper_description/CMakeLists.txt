cmake_minimum_required(VERSION 2.8.3)
project(pepper_description)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

# Instructions to install launch files
install(DIRECTORY launch/
     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# Instructions to install config files
install(DIRECTORY config/
     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

# Instructions to install launch files
install(DIRECTORY urdf/
     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)
