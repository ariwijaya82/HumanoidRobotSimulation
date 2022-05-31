CXX = g++

INCLUDE_DIRS = -I"./include" \
		  -I"$(WEBOTS_HOME)/projects/robots/robotis/darwin-op/libraries/robotis-op2/robotis/Framework/include" \
		  -I"$(WEBOTS_HOME)/include/controller/cpp" \
		  -I"$(HOME)/fuzzylite/fuzzylite"

LIBRARIES = -L"$(WEBOTS_HOME)/lib/controller" -lController -lCppController \
			-L"$(WEBOTS_HOME)/projects/robots/robotis/darwin-op/libraries/robotis-op2" -lrobotis-op2 \
			-L"${HOME}/fuzzylite/fuzzylite/release/bin" -lfuzzylite-static

CXXFLAGS = -std=c++17 $(INCLUDE_DIRS)
LDFLAGS = $(LIBRARIES)

TARGET = run
MAIN = main

OBJS = $(addsuffix .o, $(basename $(wildcard find src/*.cpp)))

all: $(TARGET)

clean:
	rm -f src/*.o $(TARGET)

$(TARGET): $(OBJS) $(MAIN).cpp
	$(CXX) $(CXXFLAGS) $(MAIN).cpp $(OBJS) -o $(TARGET) $(LIBRARIES)