CXX = g++

INCLUDE_DIRS = -I"$(WEBOTS_HOME)/include/controller/cpp"

LIBRARIES = -L"$(WEBOTS_HOME)/lib/controller" -lController -lCppController

CXXFLAGS = -std=c++17 $(INCLUDE_DIRS)
LDFLAGS = $(LIBRARIES)

TARGET = run

OBJS = $(addsuffix .o, $(basename $(wildcard find src/*.cpp)))

all: $(TARGET)

clean:
	rm -f src/*.o $(TARGET)

$(TARGET): $(OBJS) main.cpp
	$(CXX) $(CXXFLAGS) main.cpp $(OBJS) -o $(TARGET) $(LIBRARIES)