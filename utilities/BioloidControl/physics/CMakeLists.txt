cmake_minimum_required(VERSION 2.6)

PROJECT(physics CXX)

INCLUDE_DIRECTORIES(include ../main/include ode/include)

SET(BIOLOID src/bioloid.cpp src/bioloid_devices.cpp)
SET(PHYSICS src/world.cpp src/geom.cpp src/body.cpp src/joint.cpp src/servo.cpp src/sensor.cpp)
SET(MAIN    src/main.cpp src/util.cpp)
SET(GUI     src/scene.cpp)

SET(mainsrc ../main/src)
SET(CONF    
		${mainsrc}/tinystr.cpp ${mainsrc}/tinyxml.cpp ${mainsrc}/tinyxmlerror.cpp ${mainsrc}/tinyxmlparser.cpp
		${mainsrc}/configuration.cpp
)

ADD_EXECUTABLE(physics ${MAIN} ${PHYSICS} ${GUI} ${BIOLOID} ${CONF})

SET(ode_dir ode)

FIND_LIBRARY(LIB_ODE ode ${ode_dir}/ode/src/.libs)
FIND_LIBRARY(LIB_DRAWSTUFF drawstuff ${ode_dir}/drawstuff/src/.libs)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${LIB_DRAWSTUFF} ${LIB_ODE} )

TARGET_LINK_LIBRARIES(${PROJECT_NAME} boost_thread-mt boost_program_options )

TARGET_LINK_LIBRARIES(${PROJECT_NAME} GL GLU Xmu Xi X11 )
