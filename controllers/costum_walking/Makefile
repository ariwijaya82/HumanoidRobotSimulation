null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
RESOURCES_PATH = $(HOME)/developer/humanoid_robot_simulation

# include $(RESOURCES_PATH)/libraries/robotis-op2/Makefile
# include $(RESOURCES_PATH)/libraries/managers/Makefile

INCLUDE = -I.
INCLUDE += -I"$(WEBOTS_HOME)/include/controller/cpp"
INCLUDE += -I"$(WEBOTS_HOME)/include/controller/c"
INCLUDE += -I"$(RESOURCES_PATH)/libraries/managers/include"
INCLUDE += -I"$(RESOURCES_PATH)/libraries/robotis-op2/robotis/Framework/include"

LIBRARIES = -L"$(WEBOTS_HOME)/lib/controller" -lController -lCppController
LIBRARIES += -L"$(RESOURCES_PATH)/libraries/managers" -lmanagers
LIBRARIES += -L"$(RESOURCES_PATH)/libraries/robotis-op2" -lrobotis-op2 

CXX_SOURCES = $(wildcard *.cpp)

all:
	g++ $(CXX_SOURCES) $(LDFLAGS) $(INCLUDE) $(LIBRARIES) -o costum_walking 

clean_dir:
	rm costum_walking
