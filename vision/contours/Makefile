COMPILER = g++
SRC = linear_filter.cpp
EXE = main

INCLUDE = -I"/usr/include/opencv4"
LIBRARIES = -L"/usr/lib" -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_highgui

all:
	$(COMPILER) $(SRC) -o $(EXE) $(INCLUDE) $(LIBRARIES)