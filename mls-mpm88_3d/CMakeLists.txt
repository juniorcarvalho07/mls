cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (MLS_MPM_3D)



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


#find_package(X11 REQUIRED)
#include_directories(${X11_INCLUDE_DIR})
#if(NOT X11_FOUND)
#    message("ERROR: x11 not found")
#endif(NOT X11_FOUND)


set  (project_SOURCES main.cpp 
 ColorRGBA.hpp
 Point.hpp
 point3dutils.hpp
 PrintBase.hpp
 printof.hpp
 PrintOfPart.hpp
 GL_interactor.cpp
 scrInteractor.cpp
 TrackBall.cpp
 MarchingCubes.cpp
polarDecomposition.cpp
mat3d.cpp
eig3.cpp
vec3d.cpp
)
set  (project_HEADERS
Cores.h
MLS_MPM.hh
taichi.h
vector.hh
quaternion.h
GL_interactor.h
scrInteractor.h
TrackBall.h
MarchingCubes.h
LookUpTable.h
polarDecomposition.h
mat3d.h
eig3.h
vec3d.h
of/ofBase.h
of/ofBinaryIO.h
of/ofBoundaryCellCIterator2D.h
of/ofBoundaryCellIterator2D.h
of/ofBoundaryCellIterator.h
of/ofCell2D.h
of/ofCell3D.h
of/ofCell.h
of/ofCellsIterator.h
of/ofCgmPointsReader.h
of/ofCgmReader.h
of/ofDelaunay2D.h
of/ofDiDo3D.h
of/ofGeometric.h
of/of.h
of/ofIterator.h
of/ofKdTree.h
of/ofList.h
of/ofMacros.h
of/ofMarkFrat.h
of/ofMesh2D.h
of/ofMesh3D.h
of/ofMesh.h
of/ofMeshNOSurface.h
of/ofMeshOper.h
of/ofMeshSurface.h
of/ofMeshWriter.h
of/ofMyCell.h
of/ofMyDelaunay2D.h
of/ofMyGeometric.h
of/ofMyMesh.h
of/ofMyOffReader.h
of/ofMyof.h
of/ofMyTraits.h
of/ofMyVertex3D.h
of/ofMyVis.h
of/ofOffPointsReader.h
of/ofOffReader.h
of/ofOffWriter.h
of/ofOfReader.h
of/ofOfWriter.h
of/ofOperators.h
of/ofParameters.h
of/ofPgReader.h
of/ofPlane.h
of/ofPoints2DReader.h
of/ofPolyDataPointsReader.h
of/ofPolyDataReader.h
of/ofPolyDataWriter.h
of/ofPolygonBoundaries.h
of/ofPolygon.h
of/ofPolyWriter.h
of/ofReader.h
of/ofRemesh.h
of/ofRuppert2D.h
of/ofSearchDIDO.h
of/ofSing.h
of/ofSingularVertex.h
of/ofTetGen3D.h
of/ofTetGen3DReader.h
of/ofTetGenReader.h
of/ofTopology.h
of/ofTraits.h
of/ofTransfSolution.h
of/ofTReader.h
of/ofVector.h
of/ofVertex2D.h
of/ofVertex3D.h
of/ofVertex.h
of/ofVertexStarIterator2D.h
of/ofVertexStarIterator3D.h
of/ofVertexStarIterator.h
of/ofVertexStarIteratorSurface.h
of/ofVertexStarIteratorSurfaceVertex.h
of/ofVerticesIterator.h
of/ofVis2D.h
of/ofVtkPointsReader.h
of/ofvtkreader.h
of/ofVtkWriter.h
of/ofWriter.h
of/ofWrlReader.h
of/ofXmlParser.h
of/pair.h
of/ShePolyWriter.h
)



include_directories( ${CMAKE_SOURCE_DIR}/VisOf )
include_directories( ${CMAKE_SOURCE_DIR}/VisOf/Utils )
include_directories( ${CMAKE_SOURCE_DIR}/VisOf/iterFunc )
include_directories( ${CMAKE_SOURCE_DIR}/of )
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )
ADD_EXECUTABLE  (mpm_3d main.cpp scrInteractor.cpp TrackBall.cpp polarDecomposition.cpp mat3d.cpp eig3.cpp vec3d.cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -lGL  -lGLU  -lglut -lGLEW  -std=c++14 -g  -pthread   -L /lib64 ")

target_link_libraries(mpm_3d ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${CMAKE_THREAD_LIBS_INIT}   )





