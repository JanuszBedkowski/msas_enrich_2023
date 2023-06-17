cmake_minimum_required(VERSION 2.8.3)
project(mandeye_unicorn)
set(CMAKE_CXX_STANDARD 20)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  visualization_msgs
)


find_package(OpenGL REQUIRED)
if(OpenGL_FOUND)
message("OpenGL found")
message ("OPENGL_LIBRARIES: " ${OPENGL_LIBRARIES})
endif()

find_package(GLUT REQUIRED)
if(GLUT_FOUND)
message("GLUT found")
message ("GLUT_glut_LIBRARY: " ${GLUT_glut_LIBRARY})
endif()

find_package( Eigen3 REQUIRED)
if(EIGEN3_FOUND)
message("EIGEN3 found")
message ("EIGEN3_INCLUDE_DIR: " ${EIGEN3_INCLUDE_DIR})
message ("EIGEN3_INCLUDE_DIRS: " ${EIGEN3_INCLUDE_DIRS})
message ("Eigen3_DIR: " ${Eigen3_DIR})
endif()

find_package(PCL 1.5 REQUIRED)
find_library(LIVOX_LIDAR_SDK_LIBRARY  liblivox_lidar_sdk_static.a    /usr/local/lib)

include_directories(
${EIGEN3_INCLUDE_DIR}
${EIGEN3_INCLUDE_DIRS}
${Eigen3_DIR}
${PCL_INCLUDE_DIRS}
)




## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   nav_msgs#   std_msgs#   visualization_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES viewer_gl
#  CATKIN_DEPENDS nav_msgs roscpp std_msgs visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/viewer_gl.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

include_directories(src/3rd/imgui src/3rd/imgui/backends src/3rd/)

set(ImGui_Glut_src 
src/3rd/imgui/imgui.cpp 
src/3rd/imgui/imgui_demo.cpp 
src/3rd/imgui/imgui_draw.cpp 
src/3rd/imgui/imgui_widgets.cpp
src/3rd/imgui/backends/imgui_impl_glut.cpp 
src/3rd/imgui/backends/imgui_impl_opengl2.cpp
src/3rd/imgui/imgui_widgets.cpp
src/3rd/imgui/imgui_tables.cpp
)

include_directories(
	src/3rd/Fusion/Fusion
)

set(FUSION 
	src/3rd/Fusion/Fusion/FusionAhrs.c
	src/3rd/Fusion/Fusion/FusionCompass.c
	src/3rd/Fusion/Fusion/FusionOffset.c
)

include_directories(
	src/
  src/3rd/ImGuizmo
)

add_subdirectory(src/3rd/LASzip/)
include_directories(
src/3rd/LASzip/include
src/3rd/portable-file-dialogs-master
)

set(GIZMO 
src/3rd/ImGuizmo/ImGuizmo.cpp
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(mandeye_unicorn src/mandeye_unicorn.cpp ${ImGui_Glut_src} ${FUSION} ${GIZMO})


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(mandeye_unicorn ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(mandeye_unicorn ${OPENGL_LIBRARIES} ${PCL_LIBRARIES} ${GLUT_glut_LIBRARY}
 pthread glut  ${catkin_LIBRARIES} ${LIVOX_LIDAR_SDK_LIBRARY} laszip
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(base_station_unicorn src/base_station_unicorn.cpp ${ImGui_Glut_src} ${FUSION} ${GIZMO})

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(base_station_unicorn ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(base_station_unicorn ${OPENGL_LIBRARIES} ${PCL_LIBRARIES} ${GLUT_glut_LIBRARY}
 pthread glut  ${catkin_LIBRARIES} ${LIVOX_LIDAR_SDK_LIBRARY} laszip
)




#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} viewer_gl
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_viewer_gl.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
