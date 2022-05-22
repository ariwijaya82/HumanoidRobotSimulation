RESOURCES_PATH = $(HOME)/developer/humanoid_robot_simulation
PROJECT = soccer

INCLUDE = -I"./include" \
		  -I"$(RESOURCES_PATH)/project/$(PROJECT)/include" \
		  -I"$(WEBOTS_HOME)/include/controller/cpp" \
		  -I"$(WEBOTS_HOME)/include/controller/c" \
		  -I"$(RESOURCES_PATH)/framework/include" \
		  -I"/usr/include/opencv4"

LIBRARIES = -L"$(WEBOTS_HOME)/lib/controller" -lController -lCppController \
			-L"$(RESOURCES_PATH)/framework" -lframework \
			-L"/usr/lib" -lopencv_core -lopencv_highgui -lopencv_imgproc

CXX_SOURCES = $(wildcard *.cpp) $(wildcard */*.cpp)

all:
	g++ $(CXX_SOURCES) $(LDFLAGS) $(INCLUDE) $(LIBRARIES) -o run 

clean_dir:
	rm run
