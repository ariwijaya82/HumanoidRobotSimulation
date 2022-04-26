COMPILER = gcc
EXE = libtest_window
FLAG = -Wall -shared -fPIC 

QT_INCLUDE_DIR = $(WEBOTS_HOME)/include/qt
QT_LIB_DIR = $(WEBOTS_HOME)/lib/webots

INCLUDE = -I.
INCLUDE += -I"$(WEBOTS_HOME)/resources/projects/libraries/qt_utils"
INCLUDE += -I"$(WEBOTS_HOME)/include/qt/QtWidgets"
INCLUDE += -I"$(WEBOTS_HOME)/include/qt/QtGui"
INCLUDE += -I"$(WEBOTS_HOME)/include/qt/QtConcurrent"
INCLUDE += -I"$(WEBOTS_HOME)/include/qt/QtCore"
INCLUDE += -I"$(WEBOTS_HOME)/include/qt/QtNetwork"
INCLUDE += -I"$(WEBOTS_HOME)/include/qt/QtOpenGL"
INCLUDE += -I"$(WEBOTS_HOME)/include/qt/QtPrintSupport"
INCLUDE += -I"$(WEBOTS_HOME)/include/qt/QtWebEngine"
INCLUDE += -I"$(WEBOTS_HOME)/include/qt/QtWebEngineCore"
INCLUDE += -I"$(WEBOTS_HOME)/include/qt/QtWebEngineWidgets"
INCLUDE += -I"$(WEBOTS_HOME)/include/qt/QtWebSockets"


LIBRARIES = -L"$(WEBOTS_HOME)/resources/projects/libraries/qt_utils" -lqt_utils

CXX_SOURCES  = entry_points.cpp

all:
	$(COMPILER) $(FLAG) $(CXX_SOURCES) -o $(EXE).so $(INCLUDE) $(LIBRARIES)

clean:
	rm *.so