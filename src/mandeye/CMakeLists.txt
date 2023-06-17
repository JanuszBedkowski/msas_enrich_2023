cmake_minimum_required(VERSION 2.8.3)
project(mandeye)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros sensor_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package()

add_executable(mandeye src/mandeye.cpp)
target_link_libraries(mandeye ${catkin_LIBRARIES})
add_dependencies(mandeye jackal_msgs_gencpp rosserial_msgs_gencpp)
set_target_properties(mandeye
  PROPERTIES OUTPUT_NAME mandeye PREFIX "")
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
roslint_cpp()
