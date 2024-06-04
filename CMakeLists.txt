cmake_minimum_required(VERSION 3.0.2)
project(octomap_server_contact)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  octomap_server
  jsk_pcl_ros
  tf2_ros
  jsk_recognition_msgs
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES octomap_server_contact
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(octomap_server_contact src/octomap_server_contact.cpp)
target_link_libraries(octomap_server_contact
  ${catkin_LIBRARIES}
  )

add_executable(octomap_server_contact_color src/octomap_server_contact.cpp)
find_library(OCTOMAP_SERVER_COLOR_LIBRARY NAMES octomap_server_color PATHS ${octomap_server_LIBRARY_DIRS})
target_link_libraries(octomap_server_contact_color
  ${OCTOMAP_SERVER_COLOR_LIBRARY} # ${catkin_LIBRARIES}中のoctomap_serverより前に書く必要あり
  ${catkin_LIBRARIES}
  )
target_compile_definitions(octomap_server_contact_color PUBLIC COLOR_OCTOMAP_SERVER)

#############
## Install ##
#############

install(TARGETS octomap_server_contact octomap_server_contact_color
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_octomap_server_contact.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
