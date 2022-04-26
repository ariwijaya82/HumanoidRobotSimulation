#include "entry_points.hpp"
#include <iostream>

#include <core/MainApplication.hpp>
#include <gui/GenericWindow.hpp>
// #include "Viewer.hpp"

using namespace webotsQtUtils;
using namespace std;

static MainApplication *gApplication = NULL;
static GenericWindow *gViewer = NULL;

bool wbw_init() {
    cout << "Initialize entry points" << endl;
    gApplication = new MainApplication;
    
    bool result = gApplication->isInitialized();
    if (result)
        gViewer = new GenericWindow;
    return result;
}

void wbw_cleanup() {
    if (gViewer) {
        delete gViewer;
        gViewer = NULL;
    }
    if (gApplication) {
        delete gApplication;
        gApplication = NULL;
    }
}

void wbw_pre_update_gui() {
    if (gApplication && gApplication->isInitialized())
        gApplication->preUpdateGui();
}

void wbw_update_gui() {
    if (gApplication && gApplication->isInitialized())
        gApplication->updateGui();
}

void wbw_read_sensors() {
    if (gViewer && gViewer->isVisible()) 
        gViewer->readSensors();
}

void wbw_write_actuators() {
    if (gViewer && gViewer->isVisible())
        gViewer->writeActuators();
}

void wbw_show() {
    if (gViewer)
        gViewer->showWindow();
}