cmake_minimum_required(VERSION 3.5.1)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_COLOR_MAKEFILE   ON)


project(wireless_ardusub CXX C)
message(STATUS "Building ${PROJECT_NAME}...")

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
  dynamic_reconfigure
  image_utils_ros_msgs
)

#find_package(catkin REQUIRED dynamic_reconfigure)

#http://wiki.ros.org/dynamic_reconfigure/Tutorials/HowToWriteYourFirstCfgFile
generate_dynamic_reconfigure_options(
  cfg/wireless_teleop_joy.cfg
)

SET(MODULES_DIR "modules" )

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


####### HANDLE DEPENDENCIES ######

SET(DEPS_DIR ${MODULES_DIR})
SET(TMP0 ${CMAKE_CURRENT_SOURCE_DIR}/${DEPS_DIR})
GETPROJECTS(DEPS ${TMP0})
message(STATUS "Dependencies found in ${TMP0}: ${DEPS}")
FOREACH(subdir ${DEPS})
if(GIT_FOUND)
  SET(SUBPROJECT_PATH ${DEPS_DIR}/${subdir})
  SET(SUBPROJECT_FULL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/${SUBPROJECT_PATH})
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    WORKING_DIRECTORY ${SUBPROJECT_FULL_PATH}
    OUTPUT_VARIABLE _sub_build_version
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  SET(SUBPROJECT_GIT_NAME ${subdir}_${_sub_build_version})
  message(STATUS "Subproject generated git name: " ${SUBPROJECT_GIT_NAME})
  if (TARGET ${subdir})
      message(WARNING "Subproject ${SUBPROJECT_GIT_NAME} (${subdir}) has already been built!")
  else()
      add_subdirectory(${SUBPROJECT_PATH})
  endif()
else()
  message(STATUS "GIT not found")
endif()
ENDFOREACH()

######## END HANDLE DEPENDENCIES ######


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

SET(SRCPATH src/)

SET(${PROJECT_NAME}_INCLUDE_DIRS
    ${catkin_INCLUDE_DIRS}
    ${SRCPATH}lib/include
    modules
    ${${TELEROBOTICS_GIT}_INCLUDE_DIRS}
    ${${DCCOMMS_UTILS_GIT}_INCLUDE_DIRS}
    ${${MAVLINK_CPP_GIT}_INCLUDE_DIRS}
    ${${IMAGE-UTILS_GIT}_INCLUDE_DIRS}
    )


## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-fPIC -std=c++14)

include_directories(
  ${${PROJECT_NAME}_INCLUDE_DIRS}
)
SET(${PROJECT_NAME}_CPP_FILES
                        ${SRCPATH}lib/src/HROVSettings.cpp
                        ${SRCPATH}lib/src/HROVSettingsV2.cpp
                        ${SRCPATH}lib/src/HROVMoveOrder.cpp
                        ${SRCPATH}lib/src/OperatorMessage.cpp
                        ${SRCPATH}lib/src/OperatorMessageV2.cpp
                        ${SRCPATH}lib/src/HROVMessage.cpp
                        ${SRCPATH}lib/src/HROVMessageV2.cpp
                        ${SRCPATH}lib/src/TeleopOrder.cpp
                        ${SRCPATH}lib/src/nodes/Operator.cpp
                        ${SRCPATH}lib/src/nodes/ROV.cpp
                        )

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_CPP_FILES})
target_link_libraries(${PROJECT_NAME} ${DCCOMMS_UTILS_GIT} ${TELEROBOTICS_GIT})

SET(ROV_V1 rov_v1)
SET(OP_V1 op_v1)
SET(ROV_V2 rov_v2)
SET(OP_V2 op_v2)
SET(ROV_V3 rov_v3)
SET(OP_V3 op_v3)

add_executable (${ROV_V1} ${SRCPATH}main/control_v1/rov_control_v1.cpp)
target_link_libraries(${ROV_V1} ${IMAGE-UTILS_GIT} ${TELEROBOTICS_GIT} ${catkin_LIBRARIES} ${PROJECT_NAME})
add_executable (${OP_V1} ${SRCPATH}main/control_v1/operator_control_v1.cpp)
target_link_libraries(${OP_V1} ${IMAGE-UTILS_GIT}  ${TELEROBOTICS_GIT} ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable (${OP_V2} ${SRCPATH}main/control_v2/operator_control_v2.cpp)
target_link_libraries(${OP_V2} ${TELEROBOTICS_GIT} ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(${OP_V2} ${PROJECT_NAME}_gencfg)
add_executable (${ROV_V2} ${SRCPATH}main/control_v2/rov_control_v2.cpp)
target_link_libraries(${ROV_V2} ${TELEROBOTICS_GIT} ${PROJECT_NAME} ${MAVLINK_CPP_GIT})

add_executable (${OP_V3} ${SRCPATH}main/control_v3/operator_control_v3.cpp)
target_link_libraries(${OP_V3} ${TELEROBOTICS_GIT} ${DCCOMMS_UTILS_GIT} ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(${OP_V3} ${PROJECT_NAME}_gencfg)
add_executable (${ROV_V3} ${SRCPATH}main/control_v3/rov_control_v3.cpp)
target_link_libraries(${ROV_V3} ${TELEROBOTICS_GIT} ${DCCOMMS_UTILS_GIT} ${PROJECT_NAME} ${catkin_LIBRARIES} ${MAVLINK_CPP_GIT})

add_executable (NEDPublisher ${SRCPATH}main/debug_scripts/NEDPublisher.cpp)
target_link_libraries(NEDPublisher ${catkin_LIBRARIES} ${MAVLINK_CPP_GIT})

install(TARGETS ${PROJECT_NAME} ${ROVA}
        LIBRARY DESTINATION "lib"
        RUNTIME DESTINATION "bin"
        )
install(DIRECTORY "${SRCPATH}lib/include/${PROJECT_NAME}" DESTINATION "include")


catkin_package(
   LIBRARIES
    ${PROJECT_NAME}
    ${${TELEROBOTICS_GIT}_LIBRARIES}
    ${${DCCOMMS_UTILS_GIT}_LIBRARIES}
    ${${MAVLINK_CPP_GIT}_LIBRARIES}
    ${${IMAGE-UTILS_GIT}_LIBRARIES}
   INCLUDE_DIRS
    ${${PROJECT_NAME}_INCLUDE_DIRS}
   CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    actionlib_msgs
    message_runtime
    cv_bridge
    image_transport
    merbots_whrov_msgs
    visp_ros
    mavros_msgs
    image_utils_ros_msgs
    sensor_msgs
    sensor_msgs
)

roslaunch_add_file_check(launch)

message(STATUS "End ${PROJECT_NAME}")
