#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup()
{
    
    // Run init() before calling any other ofxKinectSegmentation methods
    oks.init();
    
    KW = 640;
    KH = 480;
    
	ofSetVerticalSync(true);
    
    // Allocate ofxOpenCv images for Kinect input
    rgbOfxCv.allocate(KW, KH);
    irOfxCv.allocate(KW, KH);

	gW = 400;
	gH = (gW*3)/4;
	gM = 8;
    
    
    
    /* OPENNI STUFF */
    
#if defined (TARGET_OSX)
	hardware.setup();
#endif
	recordContext.setup();
	recordDepth.setup(&recordContext);
	recordImage.setup(&recordContext);
    recordContext.toggleRegisterViewport();
	recordContext.toggleMirror();
    oniRecorder.setup(&recordContext, ONI_STREAMING);
    
    

    /* GUI STUFF */
    
	gui.setup("App Controls", gM, gM*2 + KH, 480, 480);
	gui.addPanel("Parameters", 1, false);
	gui.addPanel("Options", 1, false);
	
	//--------- PANEL 1
	gui.setWhichPanel(0);
	gui.setWhichColumn(0);
    gui.addSlider("1st-pass (edge detection) search breadth", "EDGE_SEARCH_AREA_WIDTH", 20, 0, 40, true);
    gui.addSlider("1st-pass ROI box size", "BOX_SIZE", 20, 1, 50, true);
    gui.addSlider("2nd-pass (color matching) search breadth", "COLOR_SEARCH_AREA_WIDTH", 1, 0, 5, true);
    gui.addSlider("2nd-pass ROI box size", "BOX_SIZE_2", 20, 1, 50, true);
    gui.addSlider("ROI box sparsity", "BOX_SPARSITY", 1, 1, 20, true);
    gui.addSlider("Final alpha erosion", "FINAL_ALPHA_EROSION", 2, 0, 10, true);
    gui.addSlider("Final alpha blur", "FINAL_ALPHA_BLUR", 4, 0, 20, true);
	
	//--------- PANEL 2
	gui.setWhichPanel(1);
	gui.setWhichColumn(0);
    gui.addSlider("Near threshold",		"LO_THRESHOLD", 0, 0, 10000, true);
    gui.addSlider("Far threshold",		"HI_THRESHOLD", 2000, 0, 10000, true);	
    gui.addSlider("Min contour area",	"MIN_CONTOUR_AREA",	500, 1, 4000, true);
	gui.addSlider("Max contour area",	"MAX_CONTOUR_AREA",	KW*KH, 1, KW*KH, true);
	gui.addSlider("Max number of contours",	"MAX_NUM_CONTOURS",	100, 0, 1000, true);		
		
	gui.loadSettings("controlPanelSettings.xml");
    
    kinectPrevFrameMillis = 0;
	
}




//--------------------------------------------------------------
void testApp::update()
{
	ofBackground(100, 100, 100);
	gui.update();
#ifdef TARGET_OSX // only working on Mac at the moment
	hardware.update();
#endif        
        
		
    recordContext.update();
    recordDepth.update();
    recordImage.update();		
		
	int nearThreshold		= gui.getValueI("LO_THRESHOLD");
	int farThreshold		= gui.getValueI("HI_THRESHOLD");

    rgbOfxCv.setFromPixels(recordImage.getPixels(), 640, 480);
    irOfxCv.setFromPixels(recordDepth.getDepthPixels(nearThreshold, farThreshold), 640, 480);
	irOfxCv.threshold(1);
    
    // Adjust segmentation parameters according to GUI slider values
    oks.minContourArea = gui.getValueI("MIN_CONTOUR_AREA");
    oks.maxContourArea = gui.getValueI("MAX_CONTOUR_AREA");
    oks.maxNumContours = gui.getValueI("MAX_NUM_CONTOURS");
    oks.edgeSearchAreaWidth = gui.getValueI("EDGE_SEARCH_AREA_WIDTH");
    oks.colorSearchAreaWidth = gui.getValueI("COLOR_SEARCH_AREA_WIDTH");
    oks.boxSize2 = gui.getValueI("BOX_SIZE_2");
    oks.finalAlphaErosion = gui.getValueI("FINAL_ALPHA_EROSION");
    oks.finalAlphaBlur = gui.getValueI("FINAL_ALPHA_BLUR");
    oks.boxSize = gui.getValueI("BOX_SIZE");
    oks.boxSparsity = gui.getValueI("BOX_SPARSITY");
    
    oks.getRGBA(rgbOfxCv, irOfxCv, &finalRGBAOf); // <--- Segmentation
    
    computeFrameRate();
}


//--------------------------------------------------------------
void testApp::draw()
{
    
    glColor3f(1, 1, 1);
    
    rgbOfxCv.draw(gM, gM, KW, KH);
    irOfxCv.draw(gM*2+KW, gM, KW, KH);
    finalRGBAOf.draw(gM*2+KW, gM*2+KH, KW, KH);
    gui.draw();
    
    char str[32];
    sprintf(str, "%f fps\n%d ms/f", kinectFrameRate, (int) (1000/kinectFrameRate));
    ofDrawBitmapString(str, 10, 20);
}

//--------------------------------------------------------------
void testApp::computeFrameRate(){
	float now = ofGetElapsedTimeMillis();
	float FR = 1000.0/(now - kinectPrevFrameMillis);
	float fA = 0.95; 
	float fB = 1.0-fA;
	kinectFrameRate = (fA*kinectFrameRate) + (fB*FR);
    //kinectFrameRate = FR;
	kinectPrevFrameMillis = now;
}

//--------------------------------------------------------------
void testApp::exit(){
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
    /*
	switch (key)
	{
		

	}
     */
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y) {
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button) {
	gui.mouseDragged(x, y, button);
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button) {
	gui.mousePressed(x, y, button);
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button) {
	gui.mouseReleased();
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h) {
}










