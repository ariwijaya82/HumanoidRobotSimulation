COMPILER = g++
SRC = threshold.cpp
EXE = main

INCLUDE = -I"/usr/include/opencv4"
LIBRARIES = -L"/usr/lib" -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_highgui \
						 -lopencv_videoio

all:
	$(COMPILER) $(SRC) -o $(EXE) $(INCLUDE) $(LIBRARIES)