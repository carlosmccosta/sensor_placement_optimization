cmake_minimum_required(VERSION 2.8.3)
project(active_perception)


#######################################################################################################################
## flags
#######################################################################################################################

# remove logging (ROSCONSOLE_SEVERITY_NONE=5)
# set(ROS_COMPILE_FLAGS "-DROSCONSOLE_MIN_SEVERITY=5 ${ROS_COMPILE_FLAGS}")

set(${PROJECT_NAME}_CATKIN_COMPONENTS
    geometry_msgs
    roscpp
    rosconsole
    rostime
    std_msgs
)


#######################################################################################################################
## packages
#######################################################################################################################

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
find_package(Boost REQUIRED COMPONENTS thread)



#######################################################################################################################
## catkin specific configuration
#######################################################################################################################

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        active_perception
    CATKIN_DEPENDS
        ${${PROJECT_NAME}_CATKIN_COMPONENTS}
    DEPENDS
        Boost
        GAZEBO
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
    ${GAZEBO_INCLUDE_DIRS}
)



#==============================================================================
# libraries
#==============================================================================

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
)

add_library(active_perception
    src/active_perception.cpp
)



#==============================================================================
# executables
#==============================================================================

#add_executable(active_perception_node
#    src/active_perception_node.cpp
#)



#==============================================================================
# library link
#==============================================================================

target_link_libraries(active_perception
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
)



#==============================================================================
# executables link
#==============================================================================

#target_link_libraries(active_perception_node
#    active_perception
#    ${catkin_LIBRARIES}
#)
