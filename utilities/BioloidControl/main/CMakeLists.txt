cmake_minimum_required(VERSION 2.6)

PROJECT(bioloid CXX)

INCLUDE_DIRECTORIES(include )

SET(CMAKE_C_COMPILER g++)

IF(UNIX) 
 SET(MAIN_LIBS pthread ncurses)
ENDIF(UNIX)

IF(WIN32) 
 SET(MAIN_LIBS wsock32)
 MESSAGE("MAIN_LIBS are NOT tested yet on windows")
ENDIF(WIN32)

SET(CONF src/tinystr.cpp src/tinyxml.cpp src/tinyxmlerror.cpp src/tinyxmlparser.cpp )
SET(PARSER src/y.tab.c  src/lex.yy.c src/parser.cpp src/configuration.cpp)
SET(ROBOT src/interpolation.cpp src/robot.cpp src/vecmath.cpp src/motion.cpp)
SET(UTIL  src/wrapper.cpp src/util.cpp src/global.cpp src/test.cpp src/list.cpp src/crc.cpp )
SET(PLATFORM  src/platform.cpp)
SET(COMMAND src/commands.cpp src/cmd.cpp)
SET(SOCKET  src/practicalsocket.cpp)

SET(MAIN main.cpp ${CONF} ${PARSER} ${ROBOT} ${UTIL} ${PLATFORM} ${COMMAND} ${SOCKET})

ADD_EXECUTABLE(${PROJECT_NAME} ${MAIN})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MAIN_LIBS})
