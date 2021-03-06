cmake_minimum_required(VERSION 2.8.3)
project(sensor_placement_optimization)


#######################################################################################################################
## flags
#######################################################################################################################

# remove logging (ROSCONSOLE_SEVERITY_NONE=5)
# set(ROS_COMPILE_FLAGS "-DROSCONSOLE_MIN_SEVERITY=5 ${ROS_COMPILE_FLAGS}")

set(${PROJECT_NAME}_CATKIN_COMPONENTS
    geometry_msgs
    pcl_conversions
    pcl_ros
    roscpp
    rosconsole
    rostime
    sensor_msgs
    std_msgs
)


#######################################################################################################################
## packages
#######################################################################################################################

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})
find_package(GAZEBO REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)



#######################################################################################################################
## catkin specific configuration
#######################################################################################################################

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        sensor_placement_optimization
    CATKIN_DEPENDS
        ${${PROJECT_NAME}_CATKIN_COMPONENTS}
    DEPENDS
        Boost
        Eigen3
        GAZEBO
        PCL
)



#######################################################################################################################
## build
#######################################################################################################################

#==============================================================================
# includes
#==============================================================================

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)



#==============================================================================
# libraries
#==============================================================================

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
)

add_library(sensor_placement_optimization
    src/sensor_placement_optimization.cpp
)



#==============================================================================
# executables
#==============================================================================

#add_executable(sensor_placement_optimization_node
#    src/sensor_placement_optimization_node.cpp
#)



#==============================================================================
# library link
#==============================================================================

target_link_libraries(sensor_placement_optimization
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${PCL_LIBRARIES}
)



#==============================================================================
# executables link
#==============================================================================

#target_link_libraries(sensor_placement_optimization_node
#    sensor_placement_optimization
#    ${catkin_LIBRARIES}
#)
