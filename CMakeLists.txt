cmake_minimum_required(VERSION 3.5.1)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_COLOR_MAKEFILE   ON)


SET(PROJECT_NAME_ORIG wireless_ardusub)
message(STATUS "Building ${PROJECT_NAME_ORIG}...")

#BEGIN RESOLVE GIT REVISION
#https://bravenewmethod.com/2017/07/14/git-revision-as-compiler-definition-in-build-with-cmake/
message(STATUS "Resolving GIT Version...")
set(_build_version "unknown")
find_package(Git)
if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    OUTPUT_VARIABLE _build_version
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  message( STATUS "GIT hash: ${_build_version}")
else()
  message(STATUS "GIT not found")
endif()
string(TIMESTAMP _time_stamp)
#END RESOLVE GIT REVISION

#http://stackoverflow.com/questions/7787823/cmake-how-to-get-the-name-of-all-subdirectories-of-a-directory
function(GETPROJECTS result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*/CMakeLists.txt)
  SET(dirlist "")
  FOREACH(file ${children})
        get_filename_component(subdir ${file} DIRECTORY)
        LIST(APPEND dirlist ${subdir})
  ENDFOREACH()
  SET(${result} ${dirlist} PARENT_SCOPE)
endfunction()

project(${PROJECT_NAME_ORIG} CXX C)
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
  mavros_msgs
  sensor_msgs
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
    mavros_msgs
  
    sensor_msgs
  sensor_msgs  sensor_msgs
)

roslaunch_add_file_check(launch)


###########
## Build ##
###########

####### HANDLE DEPENDENCIES ######

SET(${PROJECT_NAME}_DEPS_DIR "modules")
SET(${PROJECT_NAME}_TMP0 ${CMAKE_CURRENT_SOURCE_DIR}/${${PROJECT_NAME}_DEPS_DIR})
GETPROJECTS(${PROJECT_NAME}_DEPS ${${PROJECT_NAME}_TMP0})
message(STATUS "Dependencies found in ${${PROJECT_NAME}_TMP0}: ${${PROJECT_NAME}_DEPS}")
FOREACH(subdir ${${PROJECT_NAME}_DEPS})
add_subdirectory(${${PROJECT_NAME}_DEPS_DIR}/${subdir})
ENDFOREACH()

######## END HANDLE DEPENDENCIES ######

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

SET(${PROJECT_NAME}_INCLUDE_DIRS
    "src/lib/include"
    "modules")
SET(${PROJECT_NAME}_CPP_FILES
                        ${SRCPATH}lib/src/HROVSettings.cpp
                        ${SRCPATH}lib/src/HROVMoveOrder.cpp
                        ${SRCPATH}lib/src/OperatorMessage.cpp
                        ${SRCPATH}lib/src/HROVMessage.cpp
                        )

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_CPP_FILES})
target_link_libraries(${PROJECT_NAME} ${TELEROBOTICS_GIT})
target_include_directories(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_INCLUDE_DIRS})

SET(ROVA rov_control)
SET(OPERATOR operator_control)
add_executable (${ROVA} ${SRCPATH}main/rov_control.cpp)
target_link_libraries(${ROVA} ${TELEROBOTICS_GIT} ${IMAGE-UTILS_GIT} ${catkin_LIBRARIES} ${PROJECT_NAME})
add_executable (${OPERATOR} ${SRCPATH}main/operator_control.cpp)
target_link_libraries(${OPERATOR} ${TELEROBOTICS_GIT} ${IMAGE-UTILS_GIT} ${catkin_LIBRARIES} ${PROJECT_NAME})


install(TARGETS ${PROJECT_NAME} ${ROVA}
        LIBRARY DESTINATION "lib"
        RUNTIME DESTINATION "bin"
        )
install(DIRECTORY "${SRCPATH}lib/include/${PROJECT_NAME}" DESTINATION "include")

message(STATUS "End ${PROJECT_NAME_ORIG}")
