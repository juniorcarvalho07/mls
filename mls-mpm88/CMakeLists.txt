cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (MLS_MPM)



#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)
#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)
#########################################################
find_package (Threads)


find_package(X11 REQUIRED)
include_directories(${X11_INCLUDE_DIR})
if(NOT X11_FOUND)
    message("ERROR: x11 not found")
endif(NOT X11_FOUND)


set  (project_SOURCES main.cpp 
 ColorRGBA.hpp
)
set  (project_HEADERS
Cores.h
MLS_MPM.hh
taichi.h
)



#include_directories( ${CMAKE_SOURCE_DIR}/VisOf )
#include_directories( ${CMAKE_SOURCE_DIR}/VisOf/Utils )
#include_directories( ${CMAKE_SOURCE_DIR}/VisOf/iterFunc )
#include_directories( ${CMAKE_SOURCE_DIR}/of )
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )
ADD_EXECUTABLE  (mpm main.cpp )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -lglut -lGLEW -lX11 -std=c++14 -g  -pthread   -L /lib64 ")

target_link_libraries(mpm ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} /usr/lib/x86_64-linux-gnu/libX11.so  )





