CXX = g++

INCLUDE_DIRS = -I"./include" \
		  -I"$(WEBOTS_HOME)/projects/robots/robotis/darwin-op/libraries/robotis-op2/robotis/Framework/include" \
		  -I"$(WEBOTS_HOME)/include/controller/cpp" \
		  -I"/usr/include/opencv4"

LIBRARIES = -L"$(WEBOTS_HOME)/lib/controller" -lController -lCppController \
			-L"$(WEBOTS_HOME)/projects/robots/robotis/darwin-op/libraries/robotis-op2" -lrobotis-op2 \
			-L"/usr/lib" -lopencv_core -lopencv_highgui -lopencv_imgproc \
			-L"/usr/local/lib" -lyaml-cpp

CXXFLAGS = -std=c++17 $(INCLUDE_DIRS)
LDFLAGS = $(LIBRARIES)

TARGET = run

OBJS = $(addsuffix .o, $(basename $(wildcard find src/*.cpp)))

all: $(TARGET)

clean:
	rm -f src/*.o $(TARGET)

$(TARGET): $(OBJS) main.cpp
	$(CXX) $(CXXFLAGS) main.cpp $(OBJS) -o $(TARGET) $(LIBRARIES)