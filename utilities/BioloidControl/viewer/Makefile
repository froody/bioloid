# Project: studienarbeit
# Makefile

CPP  = g++
CC   = g++
RES  = 
OBJ  = viewer.o  $(RES)
LINKOBJ  = viewer.o $(RES)
LIBS = -L/usr/X11R6/lib -L/usr/lib -lX11 -lglut -lGL -lGLU -lm   
INCS = -I/usr/X11R6/include/ -I/usr/include 
CXXINCS =  -I include
BIN  = viewer
CXXFLAGS = $(CXXINCS)  
CFLAGS = $(INCS)
CCFLAGS = -c   
RM = rm -f

.PHONY: all all-before all-after clean clean-custom

all: all-before viewer all-after


clean: clean-custom
	${RM} $(OBJ) $(BIN)

$(BIN): $(OBJ)
	$(CPP) $(LINKOBJ) -o "viewer" $(LIBS)

viewer.o: viewer.cpp
	$(CPP) ${CCFLAGS} viewer.cpp -o viewer.o $(CXXFLAGS)
