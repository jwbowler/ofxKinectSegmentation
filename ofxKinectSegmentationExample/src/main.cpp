#include "testApp.h"
#include "ofAppGlutWindow.h"

int main() {
	ofAppGlutWindow window;
	//ofSetupOpenGL(&window, 1280, 1000, OF_WINDOW);
    ofSetupOpenGL(&window, 1304, 984, OF_WINDOW);
	ofRunApp(new testApp());
}
