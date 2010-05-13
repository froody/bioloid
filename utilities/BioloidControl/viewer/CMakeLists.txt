cmake_minimum_required(VERSION 2.6)

PROJECT(viewer CXX)

IF(UNIX) 
 SET(VIEWER viewer.cpp)
 SET(VIEWER_LIBS GL GLU Xmu Xi X11 glut)
ENDIF(UNIX)

IF(WIN32) 
 SET(VIEWER viewerwin.cpp)
 SET(VIEWER_LIBS opengl32 glu32 glaux )
 # kernel32 user32 gdi32 winspool comdlg32 
 # advapi32 shell32 ole32 oleaut32 uuid odbc32 
 MESSAGE("VIEWER_LIB are NOT SORTED OUT YET ON WINDOWS")
ENDIF(WIN32)

#MESSAGE("VIEWER_LIB are ${VIEWER_LIBS}")

ADD_EXECUTABLE(viewer ${VIEWER})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${VIEWER_LIBS})
