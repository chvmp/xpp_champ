cmake_minimum_required(VERSION 2.8.3)
project(xpp_champ)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  xpp_vis
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/champ_inverse_kinematics.cc
  src/inverse_kinematics_champ.cc
)

## URDF visualizer
add_executable(urdf_visualizer_champ src/exe/urdf_visualizer_champ.cc)
target_link_libraries(urdf_visualizer_champ
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
# Mark library for installation
install(
  TARGETS ${PROJECT_NAME} urdf_visualizer_champ
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
  DIRECTORY launch rviz 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
