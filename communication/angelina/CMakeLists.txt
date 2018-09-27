set( MOC_H
  angelinaapplication.h
  colortask.h
  mainwindow.h
  editplaygrounddialog.h
  ratiotask.h
  resultframe.h
  server.h
  task.h
  timeoutclock.h
  clock.h
  editoptionsdialog.h
)


set( SRC_H
  ${MOC_H}
  defines.h
  pillar.h
  playground.h
  qsingleton.h
)

set( SRC_CPP
  angelinaapplication.cpp
  colortask.cpp
  main.cpp
  mainwindow.cpp
  pillar.cpp
  playground.cpp
  editplaygrounddialog.cpp
  ratiotask.cpp
  resultframe.cpp
  server.cpp
  task.cpp
  timeoutclock.cpp
  clock.cpp
  editoptionsdialog.cpp
)

QT4_ADD_RESOURCES( SRC_RCS icons.qrc)
QT4_WRAP_CPP( MOC_CPP ${MOC_H} )


add_executable( angelina
	${SRC_CPP}
	${SRC_H}
	${MOC_CPP}
	${SRC_RCS}
)

target_link_libraries ( angelina
  ${QT_LIBRARIES}
)


###### Begin GIT support ######
# Version control system magic, used to create a revision number for Doxygen
IF(EXISTS ${PROJECT_SOURCE_DIR}/.git)
# Probably a GIT workspace, determine revision level
	FIND_PROGRAM(GIT_EXECUTABLE git DOC "git command line client")
	IF(GIT_EXECUTABLE)
		EXECUTE_PROCESS(COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
			WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
			OUTPUT_VARIABLE GIT_REVISION
			OUTPUT_STRIP_TRAILING_WHITESPACE)
		MESSAGE("Current revision of ${CMAKE_PROJECT_NAME} is ${GIT_REVISION}")
	ELSE(GIT_EXECUTABLE)
		SET(GIT_REVISION git)
	ENDIF(GIT_EXECUTABLE)
ENDIF(EXISTS ${PROJECT_SOURCE_DIR}/.git)
###### End GIT support ######

###### Begin doc target ######
FIND_PACKAGE(Doxygen)
IF (DOXYGEN_FOUND)
	# Parse Doxyfile.in and put output into build directory
	CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/angelina.doxy ${CMAKE_CURRENT_BINARY_DIR}/angelina.doxy)
	# adds doc target to Makefile
	ADD_CUSTOM_TARGET(angelina-doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/angelina.doxy)
ENDIF (DOXYGEN_FOUND)
###### End doc target ######

