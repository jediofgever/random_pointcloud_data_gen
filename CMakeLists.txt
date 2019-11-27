cmake_minimum_required(VERSION 2.8.3)
project(random_pointcloud_data_gen)

add_compile_options(-std=c++11)

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(gazebo 9 REQUIRED)

 
# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roscpp
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    gazebo_ros
)


find_package(PCL 1.7 REQUIRED COMPONENTS
    common
    io
    filters
    features
    segmentation
)

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})


catkin_package(
  LIBRARIES
   INCLUDE_DIRS include
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
  DEPENDS
     
)

###########
## Build ##
###########

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR}  )
include_directories(  include
${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})



add_executable(gen_node src/generator_node.cpp
                        src/RandomPCLGenerator.cpp)
target_link_libraries(gen_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS gen_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#add_dependencies(gen_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(gen_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_library(random_push SHARED plugins/RandomMoveObject.cc)
target_link_libraries(random_push ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(gt_pub SHARED plugins/SimulationPickandPlace.cc  ${SOURCES})
target_link_libraries(gt_pub ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}   )
install(TARGETS gt_pub DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(gt_pub ${PROJECT_NAME}_EXPORTED_TARGETS})
 
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
