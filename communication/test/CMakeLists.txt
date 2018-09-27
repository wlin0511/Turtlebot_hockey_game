set(MOC_H
  test.h
)

set(SRC_H
  test.h
)


set(SRC_CPP
  test.cpp
  main.cpp
)


QT4_WRAP_CPP( MOC_CPP ${MOC_H} )

add_executable( test
	${SRC_CPP}
  ${SRC_H} 
	${MOC_CPP}
)
  
target_link_libraries( test         
	${QT_LIBRARIES}
  referee 
) 

target_link_libraries( test
        ${caktin_LIBRARIES}
)

#target_link_libraries( main
#        ${caktin_LIBRARIES}
#)
