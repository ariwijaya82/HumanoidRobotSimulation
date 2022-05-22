RESOURCES_PATH = $(HOME)/developer/humanoid_robot_simulation

INCLUDE = -I"./include" \
		  -I"$(RESOURCES_PATH)/framework/include" \
		  -I"$(WEBOTS_HOME_PATH)/include/controller/cpp" \
		  -I"/usr/include/opencv4"

LIBRARIES = -L"$(WEBOTS_HOME)/lib/controller" -lController -lCppController \
			-L"$(RESOURCES_PATH)/framework" -lframework \
			-L"/usr/lib" -lopencv_core -lopencv_highgui -lopencv_imgproc \
			-L"/usr/local/lib" -lyaml-cpp

CXX_SOURCES = main.cpp $(wildcard src/*.cpp)

all:
	g++ $(CXX_SOURCES) $(INCLUDE) $(LIBRARIES) -o run

clean:
	rm run