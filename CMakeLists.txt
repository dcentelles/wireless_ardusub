cmake_minimum_required(VERSION 3.5.1)
project(wireless_ardusub CXX C)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  roslaunch
  std_srvs
  tf
  message_generation
  merbots_whrov_msgs
  cv_bridge
  image_transport
  visp_ros
)

catkin_package(
   CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    actionlib_msgs
    message_runtime
    cv_bridge
    image_transport
    visp_ros
)

roslaunch_add_file_check(launch)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_COLOR_MAKEFILE   ON)

#http://stackoverflow.com/questions/7787823/cmake-how-to-get-the-name-of-all-subdirectories-of-a-directory
MACRO(GETPROJECTS result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*/CMakeLists.txt)
  FOREACH(file ${children})
  	get_filename_component(subdir ${file} DIRECTORY)
  	LIST(APPEND dirlist ${subdir})
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()

MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
	LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()


###########
## Build ##
###########

####### HANDLE DEPENDENCIES ######

SET(${PROJECT_NAME}_DEPS_DIR "modules")
SUBDIRLIST(${PROJECT_NAME}_DEPS ${CMAKE_CURRENT_SOURCE_DIR}/${${PROJECT_NAME}_DEPS_DIR})
message(STATUS "Dependencies found: ${${PROJECT_NAME}_DEPS}")
FOREACH(subdir ${${PROJECT_NAME}_DEPS})
if (NOT TARGET ${subdir})
	add_subdirectory(${${PROJECT_NAME}_DEPS_DIR}/${subdir})
endif()
ENDFOREACH()

######## HANDLE PROJECTS ######

GETPROJECTS(projects ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "Subprojects found: ${projects}")
FOREACH(subdir ${projects})
if (NOT TARGET ${subdir})
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/${subdir})
endif()
ENDFOREACH()

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-fPIC -std=c++11)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

SET(SRCPATH src/)
include_directories(
  ${SRCPATH}lib/include
  ${catkin_INCLUDE_DIRS}
)

SET(${PROJECT_NAME}_INCLUDE_DIRS "src/lib/include")
SET(${PROJECT_NAME}_CPP_FILES
                        ${SRCPATH}lib/src/HROVSettings.cpp
                        ${SRCPATH}lib/src/HROVMoveOrder.cpp
                        ${SRCPATH}lib/src/OperatorMessage.cpp
                        ${SRCPATH}lib/src/HROVMessage.cpp
                        )

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_CPP_FILES})
target_link_libraries(${PROJECT_NAME} dccomms)
target_include_directories(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_INCLUDE_DIRS})

SET(ROVA rov_control)
add_executable (${ROVA} ${SRCPATH}main/rov_control.cpp)
target_link_libraries(${ROVA} telerobotics image-utils ${catkin_LIBRARIES} ${PROJECT_NAME})


install(TARGETS ${PROJECT_NAME} ${ROVA}
        LIBRARY DESTINATION "lib"
        RUNTIME DESTINATION "bin"
        )
install(DIRECTORY "${SRCPATH}lib/include/${PROJECT_NAME}" DESTINATION "include")
