# Project: bioloid
# Makefile

CPP  = g++
CC   = gcc
RES  = 
OBJ  = main.o src/list.o src/commands.o src/global.o src/configuration.o src/test.o src/tinystr.o src/tinyxml.o src/tinyxmlerror.o src/tinyxmlparser.o src/crc.o src/parser.o src/y.tab.o src/lex.yy.o src/cmd.o src/interpolation.o src/robot.o src/vecmath.o src/wrapper.o src/util.o src/motion.o src/platform.o src/practicalsocket.o
LINKOBJ  = main.o src/list.o src/commands.o src/global.o src/configuration.o src/test.o src/tinystr.o src/tinyxml.o src/tinyxmlerror.o src/tinyxmlparser.o src/crc.o src/parser.o src/y.tab.o src/lex.yy.o src/cmd.o src/interpolation.o src/robot.o src/vecmath.o src/wrapper.o src/util.o src/motion.o src/platform.o src/practicalsocket.o
LIBS = -lpthread -lncurses
INCS =    
CXXINCS =  -I include
BIN  = bioloid
CXXFLAGS = -Wno-deprecated $(CXXINCS)  
CFLAGS = $(INCS)
CCFLAGS = -c   
RM = rm -f

.PHONY: all all-before all-after clean clean-custom

all: all-before bioloid all-after


clean: clean-custom
	${RM} $(OBJ) $(BIN)

$(BIN): $(OBJ)
	$(CPP) $(LINKOBJ) -o "bioloid" $(LIBS)

main.o: main.cpp
	$(CPP) ${CCFLAGS} main.cpp -o main.o $(CXXFLAGS)

#src/aufgaben.o: src/aufgaben.cpp
#	$(CPP) ${CCFLAGS} src/aufgaben.cpp -o src/aufgaben.o $(CXXFLAGS)

src/tinystr.o: src/tinystr.cpp
	$(CPP) ${CCFLAGS} src/tinystr.cpp -o src/tinystr.o $(CXXFLAGS)

src/tinyxml.o: src/tinyxml.cpp
	$(CPP) ${CCFLAGS} src/tinyxml.cpp -o src/tinyxml.o $(CXXFLAGS)

src/tinyxmlerror.o: src/tinyxmlerror.cpp
	$(CPP) ${CCFLAGS} src/tinyxmlerror.cpp -o src/tinyxmlerror.o $(CXXFLAGS)

src/tinyxmlparser.o: src/tinyxmlparser.cpp
	$(CPP) ${CCFLAGS} src/tinyxmlparser.cpp -o src/tinyxmlparser.o $(CXXFLAGS)

src/crc.o: src/crc.cpp
	$(CPP) ${CCFLAGS} src/crc.cpp -o src/crc.o $(CXXFLAGS)

src/y.tab.o: src/y.tab.c
	$(CPP) ${CCFLAGS} src/y.tab.c -o src/y.tab.o $(CXXFLAGS)

src/lex.yy.o: src/lex.yy.c
	$(CPP) ${CCFLAGS} src/lex.yy.c -o src/lex.yy.o $(CXXFLAGS)

src/parser.o: src/parser.cpp
	$(CPP) ${CCFLAGS} src/parser.cpp -o src/parser.o $(CXXFLAGS)

src/cmd.o: src/cmd.cpp
	$(CPP) ${CCFLAGS} src/cmd.cpp -o src/cmd.o $(CXXFLAGS)

src/interpolation.o: src/interpolation.cpp
	$(CPP) ${CCFLAGS} src/interpolation.cpp -o src/interpolation.o $(CXXFLAGS)

src/robot.o: src/robot.cpp
	$(CPP) ${CCFLAGS} src/robot.cpp -o src/robot.o $(CXXFLAGS)

src/vecmath.o: src/vecmath.cpp
	$(CPP) ${CCFLAGS} src/vecmath.cpp -o src/vecmath.o $(CXXFLAGS)

src/wrapper.o: src/wrapper.cpp
	$(CPP) ${CCFLAGS} src/wrapper.cpp -o src/wrapper.o $(CXXFLAGS)

src/util.o: src/util.cpp
	$(CPP) ${CCFLAGS} src/util.cpp -o src/util.o $(CXXFLAGS)

src/motion.o: src/motion.cpp
	$(CPP) ${CCFLAGS} src/motion.cpp -o src/motion.o $(CXXFLAGS)

src/platform.o: src/platform.cpp
	$(CPP) ${CCFLAGS} src/platform.cpp -o src/platform.o $(CXXFLAGS)

src/commands.o: src/commands.cpp
	$(CPP) ${CCFLAGS} src/commands.cpp -o src/commands.o $(CXXFLAGS)

src/global.o: src/global.cpp
	$(CPP) ${CCFLAGS} src/global.cpp -o src/global.o $(CXXFLAGS)

src/configuration.o: src/configuration.cpp
	$(CPP) ${CCFLAGS} src/configuration.cpp -o src/configuration.o $(CXXFLAGS)

src/test.o: src/test.cpp
	$(CPP) ${CCFLAGS} src/test.cpp -o src/test.o $(CXXFLAGS)

src/list.o: src/list.cpp
	$(CPP) ${CCFLAGS} src/list.cpp -o src/list.o $(CXXFLAGS)

src/practicalsocket.o: src/practicalsocket.cpp
	$(CPP) ${CCFLAGS} src/practicalsocket.cpp -o src/practicalsocket.o $(CXXFLAGS)




