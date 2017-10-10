cmake_minimum_required(VERSION 2.8)
project(stero_velma)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
  )

