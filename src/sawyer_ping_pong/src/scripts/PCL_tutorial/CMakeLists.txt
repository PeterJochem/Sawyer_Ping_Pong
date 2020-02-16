cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(centroid)

find_package(PCL 1.2 REQUIRED)
find_package(realsense2 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} )

find_package(glfw3 3.3 QUIET)
add_subdirectory(gl)



add_executable (centroid centroid.cpp)
target_link_libraries (centroid ${PCL_LIBRARIES} )
target_link_libraries(centroid ${realsense2_LIBRARY} )
target_link_libraries(centroid ${DEPENDENCIES})
# target_link_libraries (centroid ${PCL_LIBRARIES})


