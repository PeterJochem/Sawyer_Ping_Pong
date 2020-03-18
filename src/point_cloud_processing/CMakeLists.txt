cmake_minimum_required(VERSION 2.8.3)
project(nodelet_pcl_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  pcl_ros
  tf
  tf_conversions
  geometry_msgs)

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})


add_message_files(
  FILES
  PointArray.msg
  dataPoint.msg
  )

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)


catkin_package()


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS} 
)

add_executable(cluster_extractor src/cluster_extractor.cpp)
add_dependencies(cluster_extractor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cluster_extractor ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${ARMADILLO_LIBRARIES})


add_executable(moveRobot src/moveRobot.cpp)
add_dependencies(moveRobot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(moveRobot ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(point_publisher src/point_publisher.cpp)
add_dependencies(point_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(point_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES})
