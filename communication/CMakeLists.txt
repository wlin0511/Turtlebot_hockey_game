cmake_minimum_required(VERSION 2.6)

# Project name
PROJECT( communication )


add_definitions (-fpermissive -std=c++11)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


#a dummy_target so qtcreator have access to all files
#########################################################################
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

#########################################################################


# We want to use release (debug does not work with SystemC)
set(CMAKE_BUILD_TYPE Release)

# Use some custom cmakefiles
set (CMAKE_MODULE_PATH
        /usr
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

# Use Network
set(  QT_USE_QTNETWORK TRUE )
set(  QT_USE_QTXML TRUE )

# Check for QT4
FIND_PACKAGE( Qt4 REQUIRED )
# Load QT variabls
INCLUDE( ${QT_USE_FILE} )

add_message_files(
DIRECTORY msg
FILES
pos.msg
)

add_service_files(
DIRECTORY srv
FILES
ratio.srv
ab.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

# External header
INCLUDE_DIRECTORIES( ${QT_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/hermes )

# Subdirectories
add_subdirectory (angelina)
add_subdirectory (hermes)
add_subdirectory (test)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

set(MOC_H
  test/test.h
)

set(SRC_H
  test/test.h
)


set(SRC_CPP
  test/test.cpp
  test/main.cpp
)


QT4_WRAP_CPP( MOC_CPP ${MOC_H} )

add_executable( communication_node
	${SRC_CPP}
  ${SRC_H} 
	${MOC_CPP}
)
  
target_link_libraries( communication_node         
	${QT_LIBRARIES}
  referee 
) 

target_link_libraries( communication_node
        ${catkin_LIBRARIES}
)


## Declare a C++ executable



#add_executable(communicationunication_node src/${PROJECT_NAME}/test.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(communicationunication_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

